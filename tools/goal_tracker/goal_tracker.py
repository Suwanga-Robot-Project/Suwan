#!/usr/bin/env python3
"""표준 라이브러리만 사용하는 계획 대비 목표 관리 도구."""
import argparse
from datetime import date, datetime, timezone
import html
import json
import math
import os
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parent
LEVELS = {0: '미구현', 1: '일부 구현', 2: '목표 범위 구현', 3: '목표 기준 검증 완료'}
ATTAINMENTS = {'met': '충족', 'not_met': '미충족', 'unconfirmed': '미확인'}


def check_date(value, label):
    if not isinstance(value, str):
        raise ValueError(f'{label}: YYYY-MM-DD 문자열이 필요합니다.')
    try:
        parsed = date.fromisoformat(value)
    except ValueError:
        raise ValueError(f'{label}: 유효한 YYYY-MM-DD 날짜가 필요합니다.') from None
    if parsed.isoformat() != value:
        raise ValueError(f'{label}: YYYY-MM-DD 형식이 필요합니다.')
    return value


def require_text(obj, key, label, nonempty=False):
    if not isinstance(obj.get(key), str):
        raise ValueError(f'{label}.{key}: 문자열이 필요합니다.')
    if nonempty and not obj[key].strip():
        raise ValueError(f'{label}.{key}: 빈 값은 허용하지 않습니다.')


def validate(data):
    if not isinstance(data, dict):
        raise ValueError('최상위 데이터는 JSON 객체여야 합니다.')
    check_date(data.get('as_of'), 'as_of')
    require_text(data, 'basis', 'data', nonempty=True)
    if not isinstance(data.get('goals'), list) or not data['goals']:
        raise ValueError('goals는 비어 있지 않은 목록이어야 합니다.')
    if not isinstance(data.get('history', []), list):
        raise ValueError('history는 목록이어야 합니다.')
    outcomes = data.get('additional_outcomes', [])
    if not isinstance(outcomes, list) or any(not isinstance(x, str) for x in outcomes):
        raise ValueError('additional_outcomes는 문자열 목록이어야 합니다.')
    seen = set()
    for index, g in enumerate(data['goals']):
        label = f'goals[{index}]'
        if not isinstance(g, dict):
            raise ValueError(f'{label}: JSON 객체가 필요합니다.')
        for field in ('id', 'title', 'status', 'acceptance', 'result', 'evidence', 'next_action'):
            require_text(g, field, label, nonempty=field in ('id', 'title'))
        if g['id'] in seen:
            raise ValueError('목표 ID 중복')
        seen.add(g['id'])
        if g['status'] not in ('assessed', 'unconfirmed'):
            raise ValueError('status는 assessed 또는 unconfirmed이어야 합니다.')
        if type(g.get('level')) is not int or g['level'] not in LEVELS:
            raise ValueError('level은 0~3 정수여야 합니다.')
        weight = g.get('weight')
        if type(weight) not in (int, float) or not 0 < weight < float('inf'):
            raise ValueError('weight는 양의 유한수여야 합니다.')
        attainment = g.get('attainment', 'unconfirmed')
        if not isinstance(attainment, str) or attainment not in ATTAINMENTS:
            raise ValueError(f'{label}.attainment: met, not_met, unconfirmed 중 하나여야 합니다.')
        if g['status'] == 'unconfirmed' and (g['level'] != 0 or attainment != 'unconfirmed'):
            raise ValueError(f'{g["id"]}: 미확인은 level 0과 attainment unconfirmed만 허용합니다. 평가 시 --status assessed를 지정하세요.')
        if g['level'] == 3 or attainment != 'unconfirmed':
            for field in ('acceptance', 'result', 'evidence'):
                require_text(g, field, label, nonempty=True)
        if attainment == 'met' and g['level'] != 3:
            raise ValueError(f'{g["id"]}: 충족 판정에는 검증 완료 단계(--level 3)가 필요합니다.')
        if attainment == 'not_met' and g['level'] == 3:
            raise ValueError(f'{g["id"]}: 미충족 판정과 목표 기준 검증 완료는 함께 사용할 수 없습니다.')
        if 'evaluated_at' in g:
            check_date(g['evaluated_at'], f'{label}.evaluated_at')
            if g['evaluated_at'] > data['as_of']:
                raise ValueError(f'{g["id"]}: 평가일은 보고서 기준일 이후일 수 없습니다.')


def cell(value):
    return html.escape(str(value)).replace('|', '&#124;').replace('\r\n', '\n').replace('\r', '\n').replace('\n', '<br>')


def report(data):
    validate(data)
    goals = data['goals']
    # 가중치를 먼저 정규화하여 큰 유한 가중치의 합산 오버플로를 방지합니다.
    largest = max(g['weight'] for g in goals)
    weights = {g['id']: g['weight'] / largest for g in goals}
    total = math.fsum(weights.values())
    assessed = [g for g in goals if g['status'] == 'assessed']
    points = math.fsum(weights[g['id']] * g['level'] / 3 for g in assessed)
    unknown = math.fsum(weights[g['id']] for g in goals if g['status'] == 'unconfirmed')
    coverage = math.fsum(weights[g['id']] for g in assessed) / total * 100
    lines = ['# 계획 대비 목표 달성 현황', '',
             f"기준일: {data['as_of']} / 평가 근거: {cell(data['basis'])}", '',
             f'- 확인 근거 기준 달성지수: **{points / total * 100:.1f}%**',
             f'- 평가 확인 범위: **{coverage:.1f}%**',
             f'- 미확인 목표가 모두 검증 완료인 경우의 상한: **{(points + unknown) / total * 100:.1f}%**',
             f'- 목표 기준 검증 완료: **{sum(g["level"] == 3 and g["status"] == "assessed" for g in goals)}/{len(goals)}개**', '',
             f'- 목표 충족 판정: **{sum(g.get("attainment") == "met" for g in goals)}개 충족 / {sum(g.get("attainment") == "not_met" for g in goals)}개 미충족 / {sum(g.get("attainment", "unconfirmed") == "unconfirmed" for g in goals)}개 미확인**', '',
             '> 달성지수는 단계별 점수와 가중치로 계산한 관리용 지표입니다. 공식 달성률이나 실제 성능 측정값이 아닙니다. 미확인은 실패와 구분하며, 확인된 점수에는 기여하지 않습니다. 상한은 미확인 항목에만 적용하며 이미 평가한 항목의 향후 개선은 반영하지 않습니다.', '',
             '## 목표별 현황', '',
             '| ID | 최초 목표 | 가중치 | 상태·단계 | 목표 충족 | 평가일 | 수용 기준 | 결과·변경 사항 | 증빙 | 다음 조치 |',
             '|---|---|---:|---|---|---|---|---|---|---|']
    for g in goals:
        state = '미확인' if g['status'] == 'unconfirmed' else f"{g['level']}/3 {LEVELS[g['level']]}"
        values = [g['id'], g['title'], g['weight'], state,
                  ATTAINMENTS[g.get('attainment', 'unconfirmed')], g.get('evaluated_at', '미기록'),
                  g['acceptance'], g['result'], g['evidence'], g['next_action']]
        lines.append('| ' + ' | '.join(map(cell, values)) + ' |')
    lines += ['', '## 추가 성과 (최초 목표 점수에서 제외)', '']
    lines += ['- ' + cell(item) for item in data.get('additional_outcomes', [])]
    lines += ['', '## 평가 원칙', '',
              '- 0: 미구현이 확인됨 / 1: 일부 구현 또는 대체 구현의 동등성 미검증 / 2: 목표 범위 구현, 수용 시험 미완료 / 3: 수용 기준을 충족하는 시험·성과 증빙 확보.',
              '- 최초 목표를 삭제하지 않습니다. 변경 사유는 결과에 기록하고, 승인된 변경 기준이 있으면 별도 기준선으로 관리합니다.',
              '- 최초 계획에 없는 추가 성과는 원래 목표의 누락을 상쇄하지 않습니다.',
              '- 가중치는 기본적으로 동일합니다. 중요도를 달리하려면 평가 전에 합의하고 변경 이력을 남깁니다.',
              '- 초기 평가는 문서 검토에 따른 잠정 판단입니다. 실제 장비·코드·시험 로그의 독립 검증은 수행하지 않았습니다.', '']
    return '\n'.join(lines)


def atomic_write(path, content):
    """같은 폴더의 고유 임시 파일에서 교체하여 중간 파일 노출을 방지합니다."""
    path = path.resolve()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=f'.{path.name}.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def same_file(left, right):
    return left.resolve() == right.resolve() or (
        left.exists() and right.exists() and left.samefile(right))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=ROOT / 'goals.json')
    subs = parser.add_subparsers(dest='command', required=True)
    r = subs.add_parser('report', help='Markdown 현황 생성')
    r.add_argument('--output', type=Path, help='기본값: 입력 데이터와 같은 폴더의 achievement_report.md')
    u = subs.add_parser('update', help='목표 한 건 수정 (최초 목표와 가중치는 JSON에서 관리)')
    u.add_argument('id')
    u.add_argument('--level', type=int, choices=LEVELS)
    u.add_argument('--status', choices=['assessed', 'unconfirmed'])
    u.add_argument('--attainment', choices=list(ATTAINMENTS), help='목표 충족 여부 (구현 단계와 별도)')
    for field in ('result', 'evidence', 'next_action', 'acceptance'):
        u.add_argument('--' + field.replace('_', '-'), dest=field)
    u.add_argument('--as-of', required=True, help='평가 기준일 YYYY-MM-DD')
    args = parser.parse_args()
    try:
        data = json.loads(args.data.read_text(encoding='utf-8'))
        validate(data)
        if args.command == 'report':
            output = args.output or args.data.resolve().parent / 'achievement_report.md'
            if same_file(args.data, output):
                raise ValueError('출력 경로가 입력 데이터와 같습니다. 원본 덮어쓰기를 차단했습니다.')
            atomic_write(output, report(data))
            print(f'생성: {output}')
        else:
            check_date(args.as_of, '--as-of')
            g = next((g for g in data['goals'] if g['id'] == args.id), None)
            if g is None:
                raise ValueError(f'알 수 없는 목표: {args.id}')
            before = dict(g)
            before_as_of = data['as_of']
            fields = ('level', 'status', 'attainment', 'result', 'evidence', 'next_action', 'acceptance')
            if all(getattr(args, field) is None for field in fields):
                raise ValueError('수정할 항목을 하나 이상 지정하세요.')
            if args.as_of < g.get('evaluated_at', data['as_of']):
                raise ValueError('기존 목표 평가일보다 과거로 수정할 수 없습니다. 과거 기록은 별도로 보관하세요.')
            for field in fields:
                value = getattr(args, field)
                if value is not None:
                    g[field] = value
            g['evaluated_at'] = args.as_of
            data['as_of'] = max(data['as_of'], args.as_of)
            validate(data)
            data.setdefault('history', []).append({
                'date': args.as_of, 'recorded_at': datetime.now(timezone.utc).isoformat(),
                'id': args.id, 'before': before, 'after': dict(g),
                'before_as_of': before_as_of, 'after_as_of': data['as_of']})
            atomic_write(args.data, json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
            print(f'수정: {args.id}. report 명령으로 현황을 갱신하세요.')
    except (ValueError, KeyError, OSError) as error:
        parser.exit(2, f'오류: {error}\n')


if __name__ == '__main__':
    main()
