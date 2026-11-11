"""Transactional checkpoints. Display messages are never the progress authority."""
import json
import sqlite3
from contextlib import contextmanager


class Store:
    def __init__(self, path):
        self.db = sqlite3.connect(str(path))
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript('''
            CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, data TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS events (
                seq INTEGER PRIMARY KEY AUTOINCREMENT, kind TEXT, data TEXT);
            CREATE TABLE IF NOT EXISTS requests (id TEXT PRIMARY KEY, data TEXT NOT NULL);
        ''')

    @contextmanager
    def transaction(self):
        try:
            self.db.execute('BEGIN IMMEDIATE')
            yield
            self.db.commit()
        except Exception:
            self.db.rollback()
            raise

    def save(self, task, kind):
        self.db.execute('INSERT OR REPLACE INTO tasks VALUES (?, ?)',
                        (task['id'], json.dumps(task, allow_nan=False)))
        self.event(kind, task)

    def event(self, kind, data):
        self.db.execute('INSERT INTO events(kind,data) VALUES (?,?)',
                        (kind, json.dumps(data, allow_nan=False)))

    def tasks(self):
        return {key: json.loads(data) for key, data in self.db.execute('SELECT * FROM tasks')}

    def request(self, key):
        row = self.db.execute('SELECT data FROM requests WHERE id=?', (key,)).fetchone()
        return json.loads(row[0]) if row else None

    def remember(self, key, result):
        self.db.execute('INSERT INTO requests VALUES (?,?)', (key, json.dumps(result)))

    def close(self):
        self.db.close()
