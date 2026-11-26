"""Unit tests for SurveyJournal persistence and skip summary."""
from pathlib import Path

from usv_autonomy.survey_journal import SurveyJournal


def test_survey_journal_jsonl_and_summary(tmp_path: Path):
    journal = SurveyJournal(str(tmp_path))
    journal.start('run_a', reference_path_id='path_a')
    journal.record('survey_prepare', along=0.0, state='prepare')
    journal.record('deviation_start', along=10.0, segment=0, cross=2.0, state='1')
    journal.record('deviation_end', along=40.0, segment=0, cross=0.5, state='0')
    journal.record('mission_complete', along=100.0, distance_remaining=0.0, state='complete')
    summary = journal.finalize('completed')

    jsonl = (tmp_path / 'run_a.jsonl').read_text(encoding='utf-8').strip().splitlines()
    assert len(jsonl) == 4
    assert summary['outcome'] == 'completed'
    assert summary['reference_path_id'] == 'path_a'
    assert summary['events_count'] == 4
    assert summary['skipped_segments'] == [{
        'start_along_m': 10.0,
        'end_along_m': 40.0,
        'cause': 'obstacle_rejoin',
        'disposition': 'forward_rejoin',
    }]
    assert (tmp_path / 'run_a_summary.json').exists()


def test_survey_journal_handover_closes_open_deviation(tmp_path: Path):
    journal = SurveyJournal(str(tmp_path))
    journal.start('run_b')
    journal.record('deviation_start', along=5.0)
    journal.record('wait_timeout', along=7.0)
    summary = journal.finalize('manual')
    assert summary['outcome'] == 'manual'
    assert summary['skipped_segments'][0]['cause'] == 'wait_timeout'
    assert summary['skipped_segments'][0]['disposition'] == 'manual_or_failed'
