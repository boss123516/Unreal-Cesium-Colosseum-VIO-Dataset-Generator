#!/usr/bin/env python3
"""Serve the generated local 3D Tiles with CORS and useful MIME types."""

from __future__ import annotations

import argparse
import functools
import http.server
import mimetypes
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DIRECTORY = SCRIPT_DIR / "output" / "tiles3d"


class TilesRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.end_headers()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="KAU LoD1 로컬 3D Tiles 서버")
    parser.add_argument("--directory", type=Path, default=DEFAULT_DIRECTORY)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    directory = args.directory.resolve()
    tileset = directory / "tileset.json"
    if not tileset.is_file():
        raise SystemExit(f"tileset.json이 없습니다: {tileset}")
    mimetypes.add_type("model/gltf-binary", ".glb")
    handler = functools.partial(TilesRequestHandler, directory=str(directory))
    server = http.server.ThreadingHTTPServer((args.bind, args.port), handler)
    print(f"[KAU_TILE_SERVER] directory={directory}", flush=True)
    print(
        f"[KAU_TILE_SERVER] url=http://{args.bind}:{args.port}/tileset.json",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
