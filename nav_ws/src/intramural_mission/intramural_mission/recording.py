"""Preserve every status line without putting filesystem latency in the control callback."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor


class StatusLog:
    def __init__(self,path):
        self.file=path.open('w',buffering=1)
        self.name=self.file.name
        self.pending=deque()
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='mission_log')
        self.closed=False

    def write(self,text):
        if self.closed:raise ValueError('Status log is closed')
        while self.pending and self.pending[0].done():self.pending.popleft().result()
        if len(self.pending)>=256:raise OSError('Status log writer cannot keep up')
        self.pending.append(self.executor.submit(self.file.write,text))

    def close(self):
        if self.closed:return
        self.closed=True
        self.executor.shutdown(wait=True)
        try:
            for future in self.pending:future.result()
        finally:self.file.close()
