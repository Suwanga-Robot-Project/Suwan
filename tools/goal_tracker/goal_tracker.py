#!/usr/bin/env python3
"""표준 라이브러리만 사용하는 계획 대비 목표 관리 도구."""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LEVELS = {0: '미구현', 1: '일부 구현', 2: '목표 범위 구현', 3: '목표 기준 검증 완료'}


def validate(data):
    seen = set()
    for g in data['goals']:
        if g['id'] in seen:
            raise ValueError('목표 ID 중복')
        seen.add(g['id'])
        if g['status'] not in ('assessed', 'unconfirmed'):
            raise ValueError('status는 assessed 또는 unconfirmed이어야 합니다.')
        if type(g['level']) is not int or g['level'] not in LEVELS:
            raise ValueError('level은 0~3 정수여야 합니다.')
        if type(g['weight']) not in (int, float) or not 0 < g['weight'] < float('inf'):
            raise ValueError('weight는 양의 유한수여야 합니다.')
        if g['level'] == 3 and (not g['evidence'].strip() or not g['result'].strip()):
            raise ValueError('검증 완료에는 시험 결과와 증빙이 필요합니다.')
    if not seen:
        raise ValueError('목표가 없습니다.')


def cell(value):
    return str(value).replace('|', '\\|').replace('\n', '<br>')


def report(data):
    validate(data)
    goals = data['goals']
    total = sum(g['weight'] for g in goals)
    assessed = [g for g in goals if g['status'] == 'assessed']
    points = sum(g['weight'] * g['level'] / 3 for g in assessed)
    unknown = sum(g['weight'] for g in goals if g['status'] == 'unconfirmed')
    coverage = sum(g['weight'] for g in assessed) / total * 100
    lines = ['# 계획 대비 목표 달성 현황', '',
             f"기준일: {data['as_of']} / 평가 근거: {data['basis']}", '',
             f'- 확인 근거 기준 달성지수: **{points / total * 100:.1f}%**',
             f'- 평가 확인 범위: **{coverage:.1f}%**',
             f'- 미확인 목표가 모두 검증 완료인 경우의 상한: **{(points + unknown) / total * 100:.1f}%**',
             f'- 목표 기준 검증 완료: **{sum(g["level"] == 3 and g["status"] == "assessed" for g in goals)}/{len(goals)}개**', '',
             '> 달성지수는 단계별 점수와 가중치로 계산한 관리용 지표입니다. 공식 달성률이나 실제 성능 측정값이 아닙니다. 미확인은 실패와 구분하며, 확인된 점수에는 기여하지 않습니다. 상한은 미확인 항목에만 적용하며 이미 평가한 항목의 향후 개선은 반영하지 않습니다.', '',
             '## 목표별 현황', '',
             '| ID | 최초 목표 | 가중치 | 상태·단계 | 수용 기준 | 결과·변경 사항 | 증빙 | 다음 조치 |',
             '|---|---|---:|---|---|---|---|---|']
    for g in goals:
        state = '미확인' if g['status'] == 'unconfirmed' else f"{g['level']}/3 {LEVELS[g['level']]}"
        values = [g['id'], g['title'], g['weight'], state, g['acceptance'], g['result'], g['evidence'], g['next_action']]
        lines.append('| ' + ' | '.join(map(cell, values)) + ' |')
    lines += ['', '## 추가 성과 (최초 목표 점수에서 제외)', '']
    lines += ['- ' + item for item in data.get('additional_outcomes', [])]
    lines += ['', '## 평가 원칙', '',
              '- 0: 미구현이 확인됨 / 1: 일부 구현 또는 대체 구현의 동등성 미검증 / 2: 목표 범위 구현, 수용 시험 미완료 / 3: 수용 기준을 충족하는 시험·성과 증빙 확보.',
              '- 최초 목표를 삭제하지 않습니다. 변경 사유는 결과에 기록하고, 승인된 변경 기준이 있으면 별도 기준선으로 관리합니다.',
              '- 최초 계획에 없는 추가 성과는 원래 목표의 누락을 상쇄하지 않습니다.',
              '- 가중치는 기본적으로 동일합니다. 중요도를 달리하려면 평가 전에 합의하고 변경 이력을 남깁니다.',
              '- 초기 평가는 문서 검토에 따른 잠정 판단입니다. 실제 장비·코드·시험 로그의 독립 검증은 수행하지 않았습니다.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=ROOT / 'goals.json')
    subs = parser.add_subparsers(dest='command', required=True)
    r = subs.add_parser('report', help='Markdown 현황 생성')
    r.add_argument('--output', type=Path, default=ROOT / 'achievement_report.md')
    u = subs.add_parser('update', help='목표 한 건 수정 (최초 목표와 가중치는 JSON에서 관리)')
    u.add_argument('id')
    u.add_argument('--level', type=int, choices=LEVELS)
    u.add_argument('--status', choices=['assessed', 'unconfirmed'])
    for field in ('result', 'evidence', 'next_action', 'acceptance'):
        u.add_argument('--' + field.replace('_', '-'), dest=field)
    u.add_argument('--as-of', required=True, help='평가 기준일 YYYY-MM-DD')
    args = parser.parse_args()
    try:
        data = json.loads(args.data.read_text(encoding='utf-8'))
        validate(data)
        if args.command == 'report':
            args.output.write_text(report(data), encoding='utf-8')
            print(f'생성: {args.output}')
        else:
            from datetime import date
            date.fromisoformat(args.as_of)
            g = next((g for g in data['goals'] if g['id'] == args.id), None)
            if g is None:
                raise ValueError(f'알 수 없는 목표: {args.id}')
            before = dict(g)
            for field in ('level', 'status', 'result', 'evidence', 'next_action', 'acceptance'):
                value = getattr(args, field)
                if value is not None:
                    g[field] = value
            data['as_of'] = args.as_of
            validate(data)
            data.setdefault('history', []).append({'date': args.as_of, 'id': args.id, 'before': before, 'after': dict(g)})
            temporary = args.data.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            temporary.replace(args.data)
            print(f'수정: {args.id}. report 명령으로 현황을 갱신하세요.')
    except (ValueError, KeyError, OSError) as error:
        parser.exit(2, f'오류: {error}\n')


if __name__ == '__main__':
    main()
