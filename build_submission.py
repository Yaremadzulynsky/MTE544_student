"""Package Lab 1 source, recorded CSVs, maps, and plots for LEARN.

Run from any directory with: python3 /path/to/build_submission.py
The report PDF is submitted separately and is not generated here.
"""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED


def main():
    root = Path(__file__).resolve().parent
    output = root / 'submission' / 'lab1_code.zip'
    output.parent.mkdir(exist_ok=True)
    # Include historical logs as well as the October 9 in-lab measurements.
    files = [p for p in root.iterdir()
             if p.is_file() and p.suffix in {'.py', '.sh', '.md'}]
    for folder in ('oct-9-in-lab-logs', 'sim-slam-map', 'runs', 'foxglove'):
        files.extend(p for p in (root / folder).rglob('*')
                     if p.is_file() and '__pycache__' not in p.parts)
    files.append(root / 'LICENSE')
    with ZipFile(output, 'w', ZIP_DEFLATED) as archive:
        for source in sorted(set(files)):
            archive.write(source, source.relative_to(root))
    with ZipFile(output) as archive:
        assert archive.testzip() is None, 'Archive integrity check failed'
        names = set(archive.namelist())
        for motion in ('circle', 'spiral', 'line'):
            for sensor in ('imu', 'odom', 'laser'):
                assert f'oct-9-in-lab-logs/{sensor}_content_{motion}.csv' in names
        assert {'motions.py', 'utilities.py', 'filePlotter.py',
                'oct-9-in-lab-logs/map.pgm', 'oct-9-in-lab-logs/map.yaml'} <= names
        print(f'{output}: {len(names)} files; archive integrity verified')


if __name__ == '__main__':
    main()
