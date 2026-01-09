"""
Patch for Swift to work with Python 3.12+
Fixes the "RuntimeError: no running event loop" issue in websockets
"""

import asyncio
import websockets


def patch_swift_for_python312():
    """
    Monkey-patch Swift's SwiftSocket to properly handle event loop in Python 3.12
    The issue is that websockets.serve() in line 296 of SwiftRoute.py expects
    a running event loop, but the new_event_loop() on line 288 doesn't start it.
    """
    import swift.SwiftRoute as SwiftRoute

    # Store original __init__
    original_init = SwiftRoute.SwiftSocket.__init__

    def patched_init(self, outq, inq, run):
        """Patched __init__ that properly sets up event loop before websockets.serve"""
        self.pcs = set()
        self.run = run
        self.outq = outq
        self.inq = inq
        self.USERS = set()

        # Create NEW event loop and set it for THIS thread (critical for Python 3.12)
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        started = False
        port = 53000

        # Helper to start server in the event loop context
        async def start_websocket_server(port_num):
            # websockets 12.x uses the standard (websocket, path) signature
            return await websockets.serve(self.serve, "localhost", port_num)

        while not started and port < 62000:
            try:
                # Run the async function within the loop context
                self.loop.run_until_complete(start_websocket_server(port))
                started = True
            except OSError:
                port += 1

        self.inq.put(port)
        self.loop.run_forever()

    # Apply the patch
    SwiftRoute.SwiftSocket.__init__ = patched_init
    print("Swift patched for Python 3.12+ compatibility")
