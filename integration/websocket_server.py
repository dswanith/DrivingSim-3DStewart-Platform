"""
integration.websocket_server — Async WebSocket publisher.

Hosts a WebSocket server at a fixed port. During each simulation tick,
the pipeline pushes state here. We broadcast the latest state to all
connected browser clients.

Non-blocking: If no clients are connected, or if a client is slow,
the simulation loop continues unaffected.
"""
import asyncio
import logging
from typing import Set
import websockets
import websockets.exceptions

logger = logging.getLogger(__name__)


class WebSocketPublisher:
    """
    Broadcasts simulation telemetry to connected WebSocket clients and receives commands.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8765, incoming_queue = None):
        self.host = host
        self.port = port
        self.incoming_queue = incoming_queue
        self._clients: Set = set()
        self._server = None
        self._loop = None
        self._running = False

    async def _handler(self, websocket):
        """Handle new client connections and incoming messages."""
        logger.info(f"Client connected: {websocket.remote_address}")
        self._clients.add(websocket)
        try:
            async for message in websocket:
                if self.incoming_queue is not None:
                    # Put message in queue for the main simulation loop to process
                    self.incoming_queue.put(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.remove(websocket)
            logger.info(f"Client disconnected: {websocket.remote_address}")

    async def _start_server(self):
        """Start the WebSocket server."""
        self._server = await websockets.serve(self._handler, self.host, self.port)
        logger.info(f"WebSocket server listening on ws://{self.host}:{self.port}")

    def start(self, loop: asyncio.AbstractEventLoop):
        """Start the server in the provided event loop."""
        self._loop = loop
        self._running = True
        loop.create_task(self._start_server())

    def broadcast(self, message: str):
        """
        Send a message to all connected clients.
        Non-blocking: schedules the send operation in the event loop.
        """
        if not self._running or not self._clients:
            return

        # Broadcast helper creates tasks to send to all clients concurrently
        websockets.broadcast(self._clients, message)

    async def _stop_server(self):
        """Stop the server and close all connections."""
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        for client in self._clients:
            await client.close()
        self._clients.clear()

    def stop(self):
        """Stop the server."""
        self._running = False
        if self._loop and self._server:
            # We schedule this synchronously from the main thread
            asyncio.run_coroutine_threadsafe(self._stop_server(), self._loop)
