"use client";

import { useEffect, useMemo, useState } from 'react';
import { API_BASE, apiGet, apiPost } from '../lib/api';

type Drone = {
  system_id: string;
  mode: string;
  armed: boolean;
  battery_percent: number;
  stale: boolean;
  age_seconds: number;
  role: string;
  status: string;
  a2a_url?: string;
};

type DeployResponse = {
  status: string;
  mission_id?: string;
  graph_id?: string;
  safety_report: { passed: boolean; issues: unknown[] };
  dispatch_results: unknown[];
  planning_graph?: unknown;
};

const defaultIntent = 'A구역 산악 지역 정찰 임무. 고도 120m 이하 이동 후 목표 지점 사진 촬영.';

export default function Home() {
  const [intent, setIntent] = useState(defaultIntent);
  const [drones, setDrones] = useState<Drone[]>([]);
  const [targetDrone, setTargetDrone] = useState('drone1');
  const [dispatchMode, setDispatchMode] = useState('stream');
  const [result, setResult] = useState<DeployResponse | null>(null);
  const [logs, setLogs] = useState<string[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function refreshDrones() {
    try {
      const data = await apiGet<{ items: Drone[] }>('/drones');
      setDrones(data.items || []);
      if (data.items?.[0]?.system_id) setTargetDrone(data.items[0].system_id);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  useEffect(() => {
    refreshDrones();
    const timer = setInterval(refreshDrones, 1500);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    if (!result?.mission_id) return;
    const eventSource = new EventSource(`${API_BASE}/mission/${result.mission_id}/events`);
    eventSource.onmessage = (event) => setLogs((prev) => [...prev, event.data]);
    ['connected', 'dispatch', 'ack', 'progress', 'done', 'failed', 'sent'].forEach((name) => {
      eventSource.addEventListener(name, (event) => {
        const e = event as MessageEvent;
        setLogs((prev) => [...prev, `[${name}] ${e.data}`]);
      });
    });
    return () => eventSource.close();
  }, [result?.mission_id]);

  async function submit(mode: 'plan' | 'deploy') {
    setLoading(true);
    setError(null);
    setLogs([]);
    try {
      const body = {
        intent,
        target_drones: targetDrone ? [targetDrone] : [],
        dispatch_mode: dispatchMode
      };
      const data = await apiPost<DeployResponse>(mode === 'plan' ? '/mission/plan' : '/mission/deploy', body);
      setResult(data);
      setLogs((prev) => [...prev, `[${mode}] ${JSON.stringify({ status: data.status, mission_id: data.mission_id, graph_id: data.graph_id })}`]);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setLoading(false);
    }
  }

  const graphText = useMemo(() => JSON.stringify(result?.planning_graph || {}, null, 2), [result]);

  return (
    <main className="page">
      <section className="hero">
        <div>
          <h1>AERION MIND Console</h1>
          <p>Strategic Planning Loop: Intent → Planning Graph → Safety Validation → A2A Dispatch</p>
        </div>
        <div className="badge">Layer 3 / Mind</div>
      </section>

      <section className="grid">
        <div className="card">
          <h2>임무 입력</h2>
          <div className="stack">
            <textarea value={intent} onChange={(e) => setIntent(e.target.value)} />
            <div className="row">
              <label>Target drone</label>
              <input value={targetDrone} onChange={(e) => setTargetDrone(e.target.value)} />
              <label>Dispatch</label>
              <select value={dispatchMode} onChange={(e) => setDispatchMode(e.target.value)}>
                <option value="stream">A2A SSE stream</option>
                <option value="send">A2A send</option>
                <option value="none">plan only</option>
              </select>
            </div>
            <div className="row">
              <button disabled={loading} onClick={() => submit('plan')}>계획 생성</button>
              <button disabled={loading} onClick={() => submit('deploy')}>Soma 배포</button>
              <button className="secondary" onClick={refreshDrones}>드론 갱신</button>
            </div>
            {error && <p className="small" style={{ color: '#bd1e1e' }}>{error}</p>}
          </div>
        </div>

        <div className="card">
          <h2>Connected Drones</h2>
          <table className="table">
            <thead><tr><th>ID</th><th>Mode</th><th>Battery</th><th>Age</th><th>Status</th></tr></thead>
            <tbody>
              {drones.map((drone) => (
                <tr key={drone.system_id}>
                  <td>{drone.system_id}</td>
                  <td>{drone.mode}</td>
                  <td>{drone.battery_percent}%</td>
                  <td>{drone.age_seconds.toFixed(1)}s</td>
                  <td><span className={`status ${drone.stale ? 'bad' : ''}`}>{drone.stale ? 'STALE' : drone.status}</span></td>
                </tr>
              ))}
              {drones.length === 0 && <tr><td colSpan={5}>Mock Soma heartbeat 대기 중...</td></tr>}
            </tbody>
          </table>
          <p className="small">Soma는 `/api/v1/drones/:id/heartbeat` 로 1Hz 상태를 올립니다.</p>
        </div>

        <div className="card full">
          <h2>Mission Event Stream</h2>
          <div className="log">
            {logs.map((line, idx) => <div className="log-line" key={`${idx}-${line}`}>{line}</div>)}
            {logs.length === 0 && <div className="small">배포 후 SSE 이벤트가 표시됩니다.</div>}
          </div>
        </div>

        <div className="card full">
          <h2>Planning Graph JSON</h2>
          <pre>{graphText}</pre>
        </div>
      </section>
    </main>
  );
}
