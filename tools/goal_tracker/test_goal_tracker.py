"""원본 자료를 변경하지 않는 목표 관리 회귀시험."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import goal_tracker as tracker


class GoalTrackerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.path = self.folder / 'goals.json'
        self.data = {
            'as_of': '2026-10-28', 'basis': '시험 데이터', 'history': [],
            'goals': [{
                'id': 'G01', 'title': '시험 목표', 'weight': 1,
                'status': 'assessed', 'level': 2, 'attainment': 'unconfirmed',
                'evaluated_at': '2026-10-28', 'acceptance': '오차 1 이하',
                'result': '구현', 'evidence': '시험 로그', 'next_action': '오차 측정',
            }],
        }
        self.save()

    def save(self):
        self.path.write_text(json.dumps(self.data, ensure_ascii=False), encoding='utf-8')

    def cli(self, *args):
        return subprocess.run(
            [sys.executable, str(Path(tracker.__file__).resolve()), '--data', str(self.path), *args],
            capture_output=True, text=True, cwd=self.folder,
        )

    def update(self, *args):
        return self.cli('update', 'G01', *args, '--as-of', '2026-10-29')

    def test_report_same_path_does_not_destroy_json(self):
        before = self.path.read_bytes()
        result = self.cli('report', '--output', str(self.path))
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.path.read_bytes(), before)

    def test_aliases_cannot_overwrite_input(self):
        for name, kind in [('symlink.md', 'symlink'), ('hardlink.md', 'hardlink')]:
            with self.subTest(kind=kind):
                alias = self.folder / name
                if kind == 'symlink':
                    alias.symlink_to(self.path)
                else:
                    import os
                    os.link(self.path, alias)
                self.assertEqual(self.cli('report', '--output', str(alias)).returncode, 2)
                self.assertEqual(json.loads(self.path.read_text()), self.data)

    def test_custom_data_report_stays_beside_input(self):
        self.assertEqual(self.cli('report').returncode, 0)
        self.assertTrue((self.folder / 'achievement_report.md').is_file())

    def test_empty_acceptance_cannot_be_verified(self):
        self.data['goals'][0]['acceptance'] = ''
        self.save()
        before = self.path.read_bytes()
        self.assertEqual(self.update('--level', '3').returncode, 2)
        self.assertEqual(self.path.read_bytes(), before)

    def test_unconfirmed_cannot_hold_positive_level(self):
        self.data['goals'][0].update(status='unconfirmed', level=0)
        self.save()
        self.assertEqual(self.update('--level', '3').returncode, 2)
        result = self.update('--status', 'assessed', '--level', '3', '--attainment', 'met')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_completion_and_attainment_remain_distinct(self):
        self.assertEqual(self.update('--level', '3').returncode, 0)
        goal = json.loads(self.path.read_text())['goals'][0]
        self.assertEqual(goal['attainment'], 'unconfirmed')
        self.assertEqual(self.update('--attainment', 'not_met').returncode, 2)

    def test_attainment_requires_criteria_and_verification(self):
        self.assertEqual(self.update('--attainment', 'met').returncode, 2)
        self.assertEqual(self.update('--attainment', 'not_met').returncode, 0)

    def test_bad_schema_returns_friendly_error(self):
        variants = []
        for field, value in [('evidence', None), ('id', []), ('weight', True),
                             ('level', True), ('status', []), ('attainment', [])]:
            bad = copy.deepcopy(self.data)
            bad['goals'][0][field] = value
            variants.append(bad)
        for field, value in [('goals', None), ('history', {}), ('additional_outcomes', [1])]:
            bad = copy.deepcopy(self.data)
            bad[field] = value
            variants.append(bad)
        variants.extend([[], {'goals': []}])
        for bad in variants:
            with self.subTest(data=bad):
                self.path.write_text(json.dumps(bad))
                result = self.cli('report')
                self.assertEqual(result.returncode, 2)
                self.assertNotIn('Traceback', result.stderr)

    def test_backdated_update_rejected_without_changes(self):
        before = self.path.read_bytes()
        result = self.cli('update', 'G01', '--result', '과거', '--as-of', '2026-01-01')
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.path.read_bytes(), before)

    def test_older_goal_update_preserves_global_date(self):
        self.data['as_of'] = '2026-11-01'
        self.save()
        result = self.update('--result', '새로운 결과')
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(self.path.read_text())
        self.assertEqual(data['as_of'], '2026-11-01')
        self.assertEqual(data['goals'][0]['evaluated_at'], '2026-10-29')
        entry = data['history'][0]
        self.assertEqual(entry['before'], self.data['goals'][0])
        self.assertEqual(entry['after'], data['goals'][0])
        self.assertEqual(entry['before_as_of'], entry['after_as_of'])
        self.assertTrue(entry['recorded_at'].endswith('+00:00'))

    def test_dates_and_no_change_requests_rejected(self):
        for when in ['2026-02-30', '20261029']:
            result = self.cli('update', 'G01', '--result', '내용', '--as-of', when)
            self.assertEqual(result.returncode, 2)
        self.assertEqual(self.update().returncode, 2)
        self.data['as_of'] = '2026-99-99'
        self.save()
        self.assertEqual(self.cli('report').returncode, 2)

    def test_failed_atomic_replace_preserves_original_and_cleans_temp(self):
        before = self.path.read_bytes()
        with patch.object(Path, 'replace', side_effect=OSError('교체 실패')):
            with self.assertRaises(OSError):
                tracker.atomic_write(self.path, '새 내용')
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(list(self.folder.glob('*.tmp')), [])
        self.assertEqual(list(self.folder.glob('.*.tmp')), [])

    def test_scoring_handles_unknown_and_full_completion(self):
        known = self.data['goals'][0]
        known.update(level=3, attainment='met')
        unknown = dict(known, id='G02', weight=3, status='unconfirmed', level=0,
                       attainment='unconfirmed')
        self.data['goals'].append(unknown)
        text = tracker.report(self.data)
        self.assertIn('달성지수: **25.0%**', text)
        self.assertIn('평가 확인 범위: **25.0%**', text)
        self.assertIn('상한: **100.0%**', text)
        unknown.update(status='assessed', level=3, attainment='met')
        self.assertIn('달성지수: **100.0%**', tracker.report(self.data))

    def test_large_weights_do_not_produce_nan(self):
        self.data['goals'][0]['weight'] = 1e308
        self.data['goals'].append(dict(self.data['goals'][0], id='G02'))
        text = tracker.report(self.data)
        self.assertIn('달성지수: **66.7%**', text)
        self.assertNotIn('nan%', text)

    def test_markdown_cells_escape_html_and_separators(self):
        self.assertEqual(tracker.cell('<x>|a\r\nb'), '&lt;x&gt;&#124;a<br>b')

    def test_legacy_data_without_new_fields_is_supported(self):
        goal = self.data['goals'][0]
        del goal['attainment']
        del goal['evaluated_at']
        text = tracker.report(self.data)
        self.assertIn('미기록', text)
        self.assertIn('0개 충족 / 0개 미충족 / 1개 미확인', text)


if __name__ == '__main__':
    unittest.main()
