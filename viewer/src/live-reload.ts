type ReloadHandler = () => void;
type StatusHandler = (msg: string) => void;

interface ReloadMsg {
  type: 'reload' | 'rebaking' | 'error';
  files?: string[];
  msg?: string;
}

export function connectLiveReload(opts: {
  onReload: ReloadHandler;
  onStatus: StatusHandler;
}): void {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const url = `${proto}://${location.host}/ws`;
  let retry = 0;

  const open = (): void => {
    const socket = new WebSocket(url);
    socket.addEventListener('open', () => { retry = 0; });
    socket.addEventListener('message', (ev) => {
      let msg: ReloadMsg;
      try {
        msg = JSON.parse(ev.data) as ReloadMsg;
      } catch {
        return;
      }
      if (msg.type === 'rebaking') {
        opts.onStatus(`rebaking (${(msg.files ?? []).join(', ')})…`);
      } else if (msg.type === 'reload') {
        opts.onReload();
      } else if (msg.type === 'error') {
        opts.onStatus(`bake error: ${msg.msg ?? ''}`);
      }
    });
    socket.addEventListener('close', () => {
      retry = Math.min(retry + 1, 10);
      setTimeout(open, Math.min(500 * retry, 5000));
    });
  };
  open();
}
