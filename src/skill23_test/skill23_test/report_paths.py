"""Resolve skill23_test/reports for calibrators and EKF tools.

Installed runs used to write under site-packages/reports (wrong). Prefer the
workspace source tree: <ws>/src/skill23_test/reports.
"""

from __future__ import annotations

import os
from pathlib import Path


def get_reports_dir() -> Path:
    env = os.environ.get('SKILL23_TEST_REPORTS', '').strip()
    if env:
        path = Path(env).expanduser().resolve()
        path.mkdir(parents=True, exist_ok=True)
        return path

    # Source checkout: .../src/skill23_test/skill23_test/<module>.py
    here = Path(__file__).resolve()
    for parent in here.parents:
        pkg_xml = parent / 'package.xml'
        reports = parent / 'reports'
        if not pkg_xml.is_file():
            continue
        try:
            text = pkg_xml.read_text(encoding='utf-8')
        except OSError:
            continue
        if '<name>skill23_test</name>' in text:
            reports.mkdir(parents=True, exist_ok=True)
            return reports

    # Installed: map share → <ws>/src/skill23_test/reports when possible
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('skill23_test')).resolve()
        # .../install/skill23_test/share/skill23_test → parents[3] = workspace
        if len(share.parents) >= 4 and share.parents[2].name == 'install':
            ws = share.parents[3]
            src_reports = ws / 'src' / 'skill23_test' / 'reports'
            if (ws / 'src' / 'skill23_test' / 'package.xml').is_file():
                src_reports.mkdir(parents=True, exist_ok=True)
                return src_reports
        share_reports = share / 'reports'
        share_reports.mkdir(parents=True, exist_ok=True)
        return share_reports
    except Exception:
        pass

    fallback = here.parent / 'reports'
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback
