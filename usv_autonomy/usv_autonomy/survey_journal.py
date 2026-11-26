"""Persistent survey run journal (JSONL events + structured skip summary)."""
from __future__ import annotations

import json
import os
from pathlib import Path
from time import time
from typing import Any, Dict, List, Optional


class SurveyJournal:
    """Append-only event log plus a summary written on finalize/handover."""

    HANDOVER_EVENTS = frozenset({
        'wait_timeout', 'wait_drift', 'manual', 'MANUAL_HANDOVER',
        'execution_failed', 'mission_failed',
    })

    def __init__(self, root: Optional[str] = None):
        self.root = Path(os.path.expanduser(root or '~/.ros/survey_journals'))
        self.root.mkdir(parents=True, exist_ok=True)
        self.run_id = ''
        self.path_id = ''
        self.jsonl_path: Optional[Path] = None
        self.summary_path: Optional[Path] = None
        self.events: List[Dict[str, Any]] = []
        self.skipped: List[Dict[str, Any]] = []
        self._deviation_start_along: Optional[float] = None
        self.along_final = 0.0
        self.distance_remaining = 0.0
        self.outcome = 'failed'
        self._open = False

    def start(self, run_id: str, reference_path_id: str = '') -> None:
        self.run_id = run_id or f'survey_{int(time())}'
        self.path_id = reference_path_id or self.run_id
        self.jsonl_path = self.root / f'{self.run_id}.jsonl'
        self.summary_path = self.root / f'{self.run_id}_summary.json'
        self.events = []
        self.skipped = []
        self._deviation_start_along = None
        self.along_final = 0.0
        self.distance_remaining = 0.0
        self.outcome = 'failed'
        self._open = True
        if self.jsonl_path.exists():
            self.jsonl_path.unlink()

    def record(
        self,
        event: str,
        *,
        along: float = 0.0,
        segment: int = 0,
        cross: float = 0.0,
        state: str = '',
        distance_remaining: Optional[float] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        if not self._open:
            return
        self.along_final = float(along)
        if distance_remaining is not None:
            self.distance_remaining = float(distance_remaining)
        row = {
            'stamp': time(),
            'event': event,
            'along': float(along),
            'segment': int(segment),
            'cross': float(cross),
            'state': state,
        }
        if extra:
            row.update(extra)
        self.events.append(row)
        assert self.jsonl_path is not None
        with self.jsonl_path.open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(row, ensure_ascii=False) + '\n')

        if event == 'deviation_start':
            self._deviation_start_along = float(along)
        elif event == 'deviation_end' and self._deviation_start_along is not None:
            start = self._deviation_start_along
            self.skipped.append({
                'start_along_m': start,
                'end_along_m': float(along),
                'cause': 'obstacle_rejoin',
                'disposition': 'forward_rejoin',
            })
            self._deviation_start_along = None
        elif event in self.HANDOVER_EVENTS and self._deviation_start_along is not None:
            start = self._deviation_start_along
            self.skipped.append({
                'start_along_m': start,
                'end_along_m': float(along),
                'cause': event,
                'disposition': 'manual_or_failed',
            })
            self._deviation_start_along = None

    def finalize(self, outcome: str) -> Dict[str, Any]:
        self.outcome = outcome
        summary = {
            'run_id': self.run_id,
            'mode': 'SURVEY',
            'reference_path_id': self.path_id,
            'along_final_m': self.along_final,
            'distance_remaining_m': self.distance_remaining,
            'outcome': outcome,
            'skipped_segments': list(self.skipped),
            'events_count': len(self.events),
        }
        if self.summary_path is not None:
            with self.summary_path.open('w', encoding='utf-8') as handle:
                json.dump(summary, handle, ensure_ascii=False, indent=2)
                handle.write('\n')
        self._open = False
        return summary
