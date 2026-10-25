"""Exercise the offline mapper's input validation and textureless RGB-D path."""
import pathlib
import struct
import subprocess
import sys
import tempfile

exe = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else 'build/recording_to_map').resolve()
header = struct.Struct('<IIHHH2xIII')
with tempfile.TemporaryDirectory() as directory:
    root = pathlib.Path(directory)
    frame = header.pack(1, 33, 1, 64, 48, 6144, 9216, 0) + bytes(15360)
    end = header.pack(0, 0, 255, 0, 0, 0, 0, 0)
    cases = {
        'blank': (frame * 3 + end, True),
        'truncated': (frame[:-1], False),
        'missing_eof': (frame, False),
        'bad_size': (header.pack(1, 33, 1, 64, 48, 2, 3, 0), False),
        'trailing_data': (frame + end + b'bad', False),
    }
    for name, (data, valid) in cases.items():
        source, output = root / (name + '.bin'), root / (name + '.ply')
        source.write_bytes(data)
        run = subprocess.run([str(exe), str(source), str(output)], capture_output=True, text=True, timeout=20)
        assert (run.returncode == 0) == valid, (name, run.stdout, run.stderr)
        assert output.exists() == valid, name
        if valid:
            assert 'frames=3 points=0 zero_depth=3 tracking_lost=3' in run.stdout
            assert 'element vertex 0\n' in output.read_text()
        print(name, 'OK')
