#!/usr/bin/env python3
"""Read-only source/SDK packaging. Only writes new archives under outputs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import time
import zipfile

SKIP_DIRS = {'.git', '__pycache__', '.pytest_cache', '.mypy_cache', '.venv', 'venv', 'node_modules'}
SKIP_ROOT = {'build', 'install', 'log', 'outputs', 'backups'}


def package(project, sdk, handoff, output):
    project, sdk = project.resolve(), sdk.resolve()
    if not (project/'src').is_dir() or not sdk.is_dir():
        raise RuntimeError('Project src or official SDK directory is missing')
    roots = (project, sdk)
    entries, snapshots, skipped = [], [], []
    def inside(path):
        return any(path == root or root in path.parents for root in roots)
    def walk(path, archive_path, ancestry=()):
        resolved = path.resolve(strict=True)
        if not inside(resolved):
            raise RuntimeError('External symlink requires inspection: ' + str(path))
        if resolved.is_dir():
            if resolved in ancestry:
                raise RuntimeError('Symlink cycle: ' + str(path))
            for child in sorted(path.iterdir()):
                if child.name in SKIP_DIRS:
                    continue
                if path == project and child.name in SKIP_ROOT:
                    continue
                if archive_path == 'Projects/fr5_ros2/docs' and child.name == 'migration':
                    continue
                if child.name.endswith(('.pyc', '.pyo')):
                    continue
                if child.name in {'.env', 'id_rsa', 'id_ed25519'} or (
                    child.name.startswith('.env.') and child.name not in {'.env.example', '.env.template'}):
                    skipped.append(str(child.relative_to(project)) if project in child.parents else child.name)
                    continue
                yield from walk(child, archive_path + '/' + child.name, ancestry + (resolved,))
        elif resolved.is_file():
            yield path, archive_path
        else:
            raise RuntimeError('Unsupported special file: ' + str(path))

    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6, allowZip64=True) as archive:
        def add(path, name):
            before = path.stat()
            checksum = hashlib.sha256()
            # ZIP timestamps are limited to 1980..2107. Clamp only the archive
            # metadata; preserve the actual source timestamp in the manifest.
            info = zipfile.ZipInfo.from_file(path, arcname=name, strict_timestamps=False)
            info.compress_type = zipfile.ZIP_DEFLATED
            # Materialize links as regular files; preserve executable modes.
            info.external_attr = (stat.S_IFREG | stat.S_IMODE(before.st_mode)) << 16
            with path.open('rb') as source, archive.open(info, 'w', force_zip64=True) as target:
                while True:
                    chunk = source.read(1024*1024)
                    if not chunk:
                        break
                    checksum.update(chunk); target.write(chunk)
            after = path.stat()
            signature = (before.st_size, before.st_mtime_ns, before.st_ino)
            if signature != (after.st_size, after.st_mtime_ns, after.st_ino):
                raise RuntimeError('File changed during packaging: ' + str(path))
            snapshots.append((path, signature))
            entry = {'path': name, 'bytes': before.st_size, 'sha256': checksum.hexdigest(),
                     'source_mtime_ns': before.st_mtime_ns}
            if path.is_symlink():
                entry['original_symlink'] = os.readlink(path)
            entries.append(entry)
        for root, prefix in [(project, 'Projects/fr5_ros2'), (sdk, 'SDKs/fairino')]:
            for path, name in walk(root, prefix):
                add(path, name)
        add(handoff, 'READ_ME_FIRST_ACT.md')
        pointer = project/'outputs/ikfast_hybrid_latest.txt'
        if pointer.is_file():
            record_dir = Path(pointer.read_text().strip()).resolve()
            record_dir.relative_to(project/'outputs')
            for name in ['installation.json', 'hybrid_validation.json']:
                if (record_dir/name).is_file():
                    add(record_dir/name, 'verification/'+name)
        for path, signature in snapshots:
            now = path.stat()
            if signature != (now.st_size, now.st_mtime_ns, now.st_ino):
                raise RuntimeError('Project changed while packaging; retry: ' + str(path))
        archive.writestr('MANIFEST.json', json.dumps({'created_local': time.strftime('%Y-%m-%d %H:%M:%S %z'),
            'project': str(project), 'sdk': str(sdk), 'excluded_sensitive_names': skipped,
            'symlinks_materialized': True, 'files': entries}, ensure_ascii=False, indent=2))
    return len(entries)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project', type=Path, default=Path('/data/Projects/fr5_ros2'))
    parser.add_argument('--sdk', type=Path, default=Path('/data/SDKs/fairino'))
    args = parser.parse_args()
    exports = args.project/'outputs/exports'
    exports.mkdir(parents=True, exist_ok=True)
    name = 'fr5_act_handoff_' + time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()) + '.zip'
    destination = exports/name
    count = package(args.project, args.sdk, Path(__file__).with_name('ACT_HANDOFF.md'), destination)
    checksum = hashlib.sha256()
    with destination.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            checksum.update(chunk)
    (exports/'act_export_latest.json').write_text(json.dumps({'path': str(destination),
        'sha256': checksum.hexdigest(), 'file_count': count, 'bytes': destination.stat().st_size}, indent=2))
    print('Packaged %d files: %s (%.1f MiB)' % (count, destination, destination.stat().st_size/1024**2))


if __name__ == '__main__':
    main()
