#!/usr/bin/env python3
"""
ROS 2 terrain server node.

Subscribes to TERRAIN_REQUEST from the MAVROS terrain plugin, looks up
SRTM elevation data, and publishes TERRAIN_DATA blocks back.
Also provides a TERRAIN_CHECK service for point elevation queries.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros.base import BaseNode
from mavros_msgs.msg import TerrainData, TerrainRequest
from mavros_msgs.srv import TerrainCheck, TerrainGridCheck
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.qos import QoSProfile

from .srtm import (
    compute_terrain_data_block,
    GRID_COLS,
    GRID_ROWS,
    SrtmManager,
)


@dataclass
class _PendingRequest:
    """Tracks which 4x4 blocks have been requested vs already sent."""

    lat_deg: float
    lon_deg: float
    grid_spacing: int
    mask: int
    sent_mask: int = 0

    @property
    def remaining(self) -> int:
        """Bitmask of blocks still needed."""
        return self.mask & ~self.sent_mask


class TerrainServerNode(BaseNode):
    """Serves SRTM elevation data in response to MAVLink terrain requests."""

    def __init__(self) -> None:
        super().__init__('terrain_server_node')

        self.declare_parameter(
            'terrain_data_path',
            '',
            ParameterDescriptor(
                description=(
                    'Directory for .hgt tile cache. '
                    'Empty defaults to ~/.cache/mavros/terrain/<srtm_source>/'
                )
            ),
        )
        self.declare_parameter(
            'auto_download',
            False,
            ParameterDescriptor(
                description='Automatically download missing SRTM tiles in background threads.'
            ),
        )
        self.declare_parameter(
            'download_host',
            '',
            ParameterDescriptor(
                description='Legacy alias for srtm_data_url.'
            ),
        )
        self.declare_parameter(
            'srtm_data_url',
            'https://terrain.ardupilot.org',
            ParameterDescriptor(
                description=(
                    'SRTM tile download base URL. Use http:// for Squid/nginx cacheability.'
                )
            ),
        )
        self.declare_parameter(
            'srtm_source',
            'SRTM3',
            ParameterDescriptor(
                description='SRTM dataset name (SRTM3 = 3 arc-second, SRTM1 = 1 arc-second).'
            ),
        )
        self.declare_parameter(
            'send_rate_hz',
            5.0,
            ParameterDescriptor(
                description='Rate (Hz) for sending TERRAIN_DATA blocks to the FCU.'
            ),
        )
        self.declare_parameter(
            'max_cache_tiles',
            64,
            ParameterDescriptor(
                description='Maximum number of tiles to keep in the in-memory LRU cache.'
            ),
        )
        self.declare_parameter(
            'proxy',
            '',
            ParameterDescriptor(
                description='HTTP/HTTPS proxy for tile downloads (host:port). Empty uses env vars.'
            ),
        )

        terrain_data_path = self.get_parameter('terrain_data_path').value
        auto_download = self.get_parameter('auto_download').value
        download_host = self.get_parameter('download_host').value
        srtm_data_url = download_host or self.get_parameter('srtm_data_url').value
        srtm_source = self.get_parameter('srtm_source').value
        rate_hz = self.get_parameter('send_rate_hz').value
        max_cache_tiles = self.get_parameter('max_cache_tiles').value
        proxy = self.get_parameter('proxy').value

        if rate_hz <= 0.0:
            rate_hz = 5.0

        self._auto_download = bool(auto_download)
        self._download_host = str(srtm_data_url)
        self._srtm_source = str(srtm_source)

        self._mgr = SrtmManager(
            terrain_data_path=terrain_data_path,
            auto_download=auto_download,
            srtm_data_url=srtm_data_url,
            srtm_source=srtm_source,
            max_cache_tiles=max_cache_tiles,
            proxy=proxy,
        )

        self._pending: deque[_PendingRequest] = deque()
        self._lock = threading.Lock()

        self._blocks_served = 0
        self._requests_received = 0
        self._completed_requests = 0
        self._last_request_monotonic = 0.0
        self._last_grid_check_rows = 0
        self._last_grid_check_cols = 0
        self._last_grid_check_filled = 0
        self._last_grid_check_monotonic = 0.0

        self._data_pub = self.create_publisher(
            TerrainData,
            self.get_topic('terrain', 'data'),
            QoSProfile(depth=64),
        )

        self.create_subscription(
            TerrainRequest,
            self.get_topic('terrain', 'request'),
            self._on_request,
            QoSProfile(depth=10),
        )

        self.create_service(
            TerrainCheck,
            self.get_topic('terrain', 'check'),
            self._on_check,
        )

        self.create_service(
            TerrainGridCheck,
            self.get_topic('terrain', 'grid_check'),
            self._on_grid_check,
        )

        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._on_send_tick)

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            QoSProfile(depth=10),
        )
        self._diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics)
        self._summary_log_timer = self.create_timer(60.0, self._log_summary)

        self.get_logger().info(
            f'Terrain server ready  path={terrain_data_path or "(none)"}'
            f'  auto_download={auto_download}  rate={rate_hz:.1f} Hz'
            f'  proxy={proxy or "(env)"}'
        )

    # ---------------------------------------------------------------- callbacks

    def _on_request(self, msg: TerrainRequest) -> None:
        lat_deg = msg.latitude
        lon_deg = msg.longitude

        with self._lock:
            for req in self._pending:
                if (
                    req.lat_deg == lat_deg
                    and req.lon_deg == lon_deg
                    and req.grid_spacing == msg.grid_spacing
                ):
                    req.mask = msg.mask
                    req.sent_mask &= msg.mask
                    self.get_logger().debug(
                        f'Updated pending request lat={lat_deg:.7f}'
                        f' lon={lon_deg:.7f} mask=0x{msg.mask:016x}'
                    )
                    return

            self._pending.append(_PendingRequest(lat_deg, lon_deg, msg.grid_spacing, msg.mask))
            self._requests_received += 1
            self._last_request_monotonic = time.monotonic()
            count = self._requests_received

        self.get_logger().debug(
            f'TERRAIN_REQUEST #{count} lat={lat_deg:.7f} lon={lon_deg:.7f}'
            f' spacing={msg.grid_spacing} mask=0x{msg.mask:016x}'
        )

    def _on_check(
        self,
        request: TerrainCheck.Request,
        response: TerrainCheck.Response,
    ) -> TerrainCheck.Response:
        elev = self._mgr.lookup_elevation(request.latitude, request.longitude)
        if elev is None:
            response.success = False
            response.terrain_height = 0.0
            self.get_logger().debug(
                f'Check: no data at ({request.latitude:.7f}, {request.longitude:.7f})'
            )
        else:
            response.success = True
            response.terrain_height = float(elev)
        return response

    def _on_grid_check(
        self,
        request: TerrainGridCheck.Request,
        response: TerrainGridCheck.Response,
    ) -> TerrainGridCheck.Response:
        res_deg = request.resolution_deg
        if res_deg <= 0.0:
            response.success = False
            return response

        rows = int(math.ceil((request.max_latitude - request.min_latitude) / res_deg)) + 1
        cols = int(math.ceil((request.max_longitude - request.min_longitude) / res_deg)) + 1

        max_cells = 50000
        if rows * cols > max_cells:
            self.get_logger().warn(
                f'Grid check too large: {rows}x{cols}={rows * cols} > {max_cells}'
            )
            response.success = False
            return response

        elevations = np.full(rows * cols, float('nan'), dtype=np.float32)
        filled = 0
        for r in range(rows):
            lat = request.min_latitude + r * res_deg
            for c in range(cols):
                lon = request.min_longitude + c * res_deg
                elev = self._mgr.lookup_elevation(lat, lon)
                if elev is not None:
                    elevations[r * cols + c] = float(elev)
                    filled += 1

        response.success = filled > 0
        response.rows = rows
        response.cols = cols
        response.elevations = elevations.tolist()

        with self._lock:
            self._last_grid_check_rows = rows
            self._last_grid_check_cols = cols
            self._last_grid_check_filled = filled
            self._last_grid_check_monotonic = time.monotonic()

        self.get_logger().debug(
            f'Grid check: {rows}x{cols} cells, {filled} filled'
            f' ({request.min_latitude:.5f},{request.min_longitude:.5f})'
            f' to ({request.max_latitude:.5f},{request.max_longitude:.5f})'
        )
        return response

    # ---------------------------------------------------------------- timer

    def _on_send_tick(self) -> None:
        with self._lock:
            while self._pending and self._pending[0].remaining == 0:
                self._pending.popleft()

            if not self._pending:
                return

            req = self._pending[0]
            needed = req.remaining
            lat_deg = req.lat_deg
            lon_deg = req.lon_deg
            grid_spacing = req.grid_spacing

        # Convert to degE7 for compute_terrain_data_block (MAVLink wire format)
        lat_e7 = round(lat_deg * 1e7)
        lon_e7 = round(lon_deg * 1e7)

        for bit in range(GRID_COLS * GRID_ROWS):
            if not (needed & (1 << bit)):
                continue

            data = compute_terrain_data_block(self._mgr, lat_e7, lon_e7, grid_spacing, bit)
            if data is None:
                # Tile not available yet (may be downloading in background)
                continue

            msg = TerrainData()
            msg.latitude = lat_deg
            msg.longitude = lon_deg
            msg.grid_spacing = grid_spacing
            msg.gridbit = bit
            msg.data = data

            self._data_pub.publish(msg)

            with self._lock:
                req.sent_mask |= 1 << bit
                self._blocks_served += 1
                if req.remaining == 0:
                    self._completed_requests += 1
                    self.get_logger().debug(
                        f'Completed terrain request lat={lat_deg:.7f}'
                        f' lon={lon_deg:.7f}'
                        f' ({self._blocks_served} blocks served total)'
                    )

            return

    # ----------------------------------------------------------- diagnostics

    def _snapshot_state(self):
        with self._lock:
            tiles_indexed = len(self._mgr._file_index)
            tiles_in_memory = len(self._mgr._cache)
            requests = self._requests_received
            blocks = self._blocks_served
            pending = len(self._pending)
            completed = self._completed_requests
            last_req_mono = self._last_request_monotonic
            gc_rows = self._last_grid_check_rows
            gc_cols = self._last_grid_check_cols
            gc_filled = self._last_grid_check_filled
            gc_mono = self._last_grid_check_monotonic
        return (
            tiles_indexed, tiles_in_memory, requests, blocks, pending,
            completed, last_req_mono, gc_rows, gc_cols, gc_filled, gc_mono,
        )

    def _publish_diagnostics(self) -> None:
        (tiles_indexed, tiles_in_memory, requests, blocks, pending,
         completed, last_req_mono,
         gc_rows, gc_cols, gc_filled, _gc_mono) = self._snapshot_state()

        now_mono = time.monotonic()
        last_request_age = -1.0
        if last_req_mono > 0.0:
            last_request_age = now_mono - last_req_mono

        status = DiagnosticStatus()
        status.name = 'terrain_server_node: SRTM'
        status.hardware_id = 'terrain_server_node'

        def kv(k: str, v: str) -> KeyValue:
            keyvalue = KeyValue()
            keyvalue.key = k
            keyvalue.value = v
            return keyvalue

        status.values = [
            kv('tiles_indexed', str(tiles_indexed)),
            kv('tiles_in_memory', str(tiles_in_memory)),
            kv('auto_download', 'true' if self._auto_download else 'false'),
            kv('download_host', self._download_host),
            kv('srtm_source', self._srtm_source),
            kv('requests_received', str(requests)),
            kv('completed_requests', str(completed)),
            kv('blocks_served', str(blocks)),
            kv('pending_requests', str(pending)),
            kv('last_grid_check_rows', str(gc_rows)),
            kv('last_grid_check_cols', str(gc_cols)),
            kv('last_grid_check_filled', str(gc_filled)),
            kv('last_request_age_s', f'{last_request_age:.1f}'),
        ]

        if not self._auto_download and tiles_indexed == 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'No SRTM tiles indexed and auto_download disabled'
        elif tiles_indexed == 0 and tiles_in_memory == 0 and blocks == 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'No SRTM data loaded yet'
        elif last_request_age > 60.0:
            status.level = DiagnosticStatus.STALE
            status.message = (
                f'No FCU terrain requests received in the last {last_request_age:.0f}s'
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                f'{tiles_indexed} tiles indexed; {blocks} blocks served '
                f'({pending} pending)'
            )

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(status)
        self._diagnostics_pub.publish(arr)

    def _log_summary(self) -> None:
        (tiles_indexed, tiles_in_memory, requests, blocks, pending,
         completed, _last_req_mono,
         gc_rows, gc_cols, gc_filled, _gc_mono) = self._snapshot_state()

        if requests == 0 and gc_filled == 0:
            return

        self.get_logger().info(
            f'FCU terrain: tiles={tiles_indexed} (mem={tiles_in_memory}) '
            f'requests={requests} completed={completed} pending={pending} '
            f'blocks_served={blocks} last_grid={gc_rows}x{gc_cols}/{gc_filled}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
