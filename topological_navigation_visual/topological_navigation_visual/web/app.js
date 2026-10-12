/*
 * Topological navigation web app (React + cytoscape.js, no build step).
 *
 * The front end is stateless: it renders whatever the backend pushes via
 * Server-Sent Events (/api/events) and sends commands with POST requests.
 * Only purely visual state (hovered node, viewport) lives in the browser,
 * so all open clients always show the same map, robot and goal.
 */
(function () {
  'use strict';

  const html = htm.bind(React.createElement);
  const { useEffect, useRef, useState } = React;

  const COLORS = {
    node: '#2563eb',
    roboflow: '#16a34a',
    edge: '#9ca3af',
    route: '#f97316',
    currentEdge: '#dc2626',
    currentNode: '#0ea5e9',
    destination: '#dc2626',
    currentGoal: '#f59e0b',
    robot: '#111827',
  };

  // Map metres -> graph units such that a typical edge is ~80 units long.
  function computeScale(graph) {
    const pos = {};
    graph.nodes.forEach((n) => { pos[n.name] = n; });
    const lengths = [];
    graph.edges.forEach((e) => {
      const a = pos[e.source];
      const b = pos[e.target];
      if (a && b) {
        const d = Math.hypot(a.x - b.x, a.y - b.y);
        if (d > 1e-6) lengths.push(d);
      }
    });
    if (!lengths.length) return 50;
    lengths.sort((a, b) => a - b);
    return 80 / lengths[Math.floor(lengths.length / 2)];
  }

  function toGraph(x, y, scale) {
    return { x: x * scale, y: -y * scale };
  }

  async function postJson(url, body) {
    const res = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    let data = {};
    try { data = await res.json(); } catch (e) { /* ignore */ }
    if (!res.ok || data.ok === false) {
      throw new Error(data.message || ('HTTP ' + res.status));
    }
    return data;
  }

  function fmt(value, digits, unit) {
    if (value === null || value === undefined || Number.isNaN(value)) return '\u2013';
    return value.toFixed(digits) + (unit ? ' ' + unit : '');
  }

  const CY_STYLE = [
    {
      selector: 'node.topo',
      style: {
        'background-color': COLORS.node,
        width: 22, height: 22,
        label: 'data(name)',
        'font-size': 10,
        color: '#374151',
        'text-valign': 'bottom',
        'text-margin-y': 3,
        'text-background-color': '#ffffff',
        'text-background-opacity': 0.7,
        'text-background-padding': 1,
        'min-zoomed-font-size': 7,
        'border-width': 0,
      },
    },
    { selector: 'node.topo.roboflow', style: { 'background-color': COLORS.roboflow } },
    { selector: 'node.topo.route', style: { 'border-width': 3, 'border-color': COLORS.route } },
    { selector: 'node.topo.current-goal', style: { 'border-width': 5, 'border-color': COLORS.currentGoal } },
    { selector: 'node.topo.destination', style: { 'border-width': 5, 'border-color': COLORS.destination, width: 28, height: 28 } },
    { selector: 'node.topo.current', style: { 'border-width': 5, 'border-color': COLORS.currentNode } },
    { selector: 'node.topo.hover', style: { 'overlay-opacity': 0.15, 'overlay-color': '#000000' } },
    {
      selector: 'edge.topo',
      style: {
        width: 2,
        'line-color': COLORS.edge,
        'target-arrow-color': COLORS.edge,
        'target-arrow-shape': 'triangle',
        'arrow-scale': 0.8,
        'curve-style': 'bezier',
      },
    },
    {
      selector: 'edge.topo.route',
      style: {
        width: 5, 'line-color': COLORS.route, 'target-arrow-color': COLORS.route, 'z-index': 5,
      },
    },
    {
      selector: 'edge.topo.current-edge',
      style: {
        width: 6, 'line-color': COLORS.currentEdge, 'target-arrow-color': COLORS.currentEdge, 'z-index': 6,
      },
    },
    {
      selector: 'node.robot',
      style: {
        'background-color': COLORS.robot, width: 26, height: 26,
        'border-width': 3, 'border-color': '#ffffff', 'z-index': 20, events: 'no',
      },
    },
    {
      selector: 'node.robot-head',
      style: { width: 1, height: 1, opacity: 0, events: 'no' },
    },
    {
      selector: 'edge.robot-dir',
      style: {
        width: 4, 'line-color': COLORS.robot, 'target-arrow-color': COLORS.robot,
        'target-arrow-shape': 'triangle', 'curve-style': 'straight', 'z-index': 20, events: 'no',
      },
    },
  ];

  // ------------------------------------------------------------------
  // Server connection: map + state are pushed by the backend.
  // ------------------------------------------------------------------
  function useServerState() {
    const [mapSnap, setMapSnap] = useState(null);
    const [state, setState] = useState(null);
    const [connected, setConnected] = useState(false);
    useEffect(() => {
      const es = new EventSource('api/events');
      es.addEventListener('map', (e) => setMapSnap(JSON.parse(e.data)));
      es.addEventListener('state', (e) => { setState(JSON.parse(e.data)); setConnected(true); });
      es.onopen = () => setConnected(true);
      es.onerror = () => setConnected(false);
      return () => es.close();
    }, []);
    return { mapSnap, state, connected };
  }

  // ------------------------------------------------------------------
  // Map view (cytoscape)
  // ------------------------------------------------------------------
  function MapView({ mapSnap, state, onNodeClick, cyRef }) {
    const containerRef = useRef(null);
    const scaleRef = useRef(50);
    const nodesRef = useRef({});
    const clickRef = useRef(onNodeClick);
    const [hover, setHover] = useState(null);
    clickRef.current = onNodeClick;

    // Create the cytoscape instance once.
    useEffect(() => {
      const cy = cytoscape({
        container: containerRef.current,
        style: CY_STYLE,
        autoungrabify: true,
        boxSelectionEnabled: false,
        wheelSensitivity: 0.3,
        minZoom: 0.02,
        maxZoom: 20,
      });
      cy.on('mouseover', 'node.topo', (evt) => {
        const n = evt.target;
        n.addClass('hover');
        const p = n.renderedPosition();
        setHover({ name: n.data('name'), x: p.x, y: p.y });
        containerRef.current.style.cursor = 'pointer';
      });
      cy.on('mouseout', 'node.topo', (evt) => {
        evt.target.removeClass('hover');
        setHover(null);
        containerRef.current.style.cursor = '';
      });
      cy.on('viewport', () => setHover(null));
      cy.on('tap', 'node.topo', (evt) => clickRef.current(evt.target.data('name')));
      cyRef.current = cy;
      return () => { cy.destroy(); cyRef.current = null; };
    }, []);

    // Rebuild graph elements whenever the backend map changes.
    const map = mapSnap && mapSnap.map;
    useEffect(() => {
      const cy = cyRef.current;
      if (!cy) return;
      cy.elements().remove();
      nodesRef.current = {};
      if (!map) return;
      const scale = computeScale(map);
      scaleRef.current = scale;
      const elements = [];
      map.nodes.forEach((n) => {
        nodesRef.current[n.name] = n;
        elements.push({
          group: 'nodes',
          data: { id: 'n:' + n.name, name: n.name },
          position: toGraph(n.x, n.y, scale),
          classes: 'topo' + (n.roboflow ? ' roboflow' : ''),
        });
      });
      map.edges.forEach((e, i) => {
        elements.push({
          group: 'edges',
          data: { id: 'e:' + i + ':' + e.id, edgeId: e.id, source: 'n:' + e.source, target: 'n:' + e.target },
          classes: 'topo',
        });
      });
      cy.add(elements);
      cy.fit(undefined, 40);
    }, [mapSnap && mapSnap.map_version]);

    // Apply live robot / navigation state.
    useEffect(() => {
      const cy = cyRef.current;
      if (!cy || !state || !map) return;
      const nav = state.navigation;
      const robot = state.robot;
      const routeNodes = new Set(nav.route || []);
      const routeEdges = new Set(nav.route_edges || []);
      cy.batch(() => {
        cy.nodes('.topo').forEach((n) => {
          const name = n.data('name');
          n.toggleClass('current', name === robot.current_node);
          n.toggleClass('destination', name === nav.destination);
          n.toggleClass('current-goal', name === nav.current_goal);
          n.toggleClass('route', routeNodes.has(name));
        });
        cy.edges('.topo').forEach((e) => {
          const id = e.data('edgeId');
          e.toggleClass('route', routeEdges.has(id));
          e.toggleClass('current-edge', nav.active && id === nav.current_edge);
        });

        const pose = robot.pose;
        let r = cy.getElementById('__robot');
        if (!pose) {
          cy.remove('.robot, .robot-head, .robot-dir');
          return;
        }
        const scale = scaleRef.current;
        const p = toGraph(pose.x, pose.y, scale);
        const h = { x: p.x + 30 * Math.cos(pose.yaw), y: p.y - 30 * Math.sin(pose.yaw) };
        if (r.empty()) {
          cy.add([
            { group: 'nodes', data: { id: '__robot' }, position: p, classes: 'robot', selectable: false },
            { group: 'nodes', data: { id: '__robot_head' }, position: h, classes: 'robot-head', selectable: false },
            { group: 'edges', data: { id: '__robot_dir', source: '__robot', target: '__robot_head' }, classes: 'robot-dir' },
          ]);
        } else {
          r.position(p);
          cy.getElementById('__robot_head').position(h);
        }
      });
    }, [state, mapSnap && mapSnap.map_version]);

    const hoverNode = hover && nodesRef.current[hover.name];
    return html`
      <div class="cy" ref=${containerRef}></div>
      ${!map && html`<div class="overlay">Waiting for topological map\u2026</div>`}
      ${hoverNode && html`<${NodeTooltip} node=${hoverNode} x=${hover.x} y=${hover.y}
          outgoing=${map.edges.filter((e) => e.source === hoverNode.name)} />`}
    `;
  }

  function ConfirmModal({ message, onConfirm, onCancel }) {
    return html`
      <div class="modal-backdrop" onClick=${onCancel}>
        <div class="modal" onClick=${(e) => e.stopPropagation()}>
          <p>${message}</p>
          <div class="modal-actions">
            <button class="btn" onClick=${onCancel}>Cancel</button>
            <button class="btn danger" onClick=${onConfirm}>Confirm</button>
          </div>
        </div>
      </div>`;
  }

  function NodeTooltip({ node, x, y, outgoing }) {
    const props = node.properties || {};
    const hasProps = Object.keys(props).length > 0;
    const edgeText = outgoing.map((e) => e.target + (e.action ? ' (' + e.action + ')' : '')).join(', ');
    return html`
      <div class="tooltip" style=${{ left: (x + 16) + 'px', top: (y + 16) + 'px' }}>
        <h4>${node.name}</h4>
        <div class="muted">x ${fmt(node.x, 2)}, y ${fmt(node.y, 2)}, yaw ${fmt(node.yaw, 2)}
          ${node.roboflow ? ' \u00b7 roboflow enabled' : ''}</div>
        ${edgeText && html`<div class="muted">\u2192 ${edgeText}</div>`}
        ${hasProps
          ? html`<pre>${JSON.stringify(props, null, 2)}</pre>`
          : html`<div class="muted">no properties</div>`}
        <div class="muted">Click to send the robot here</div>
      </div>`;
  }

  // ------------------------------------------------------------------
  // Side panel
  // ------------------------------------------------------------------
  function KV({ rows }) {
    return html`<dl class="kv">${rows.map(([k, v]) => html`
      <dt key=${k + '-k'}>${k}</dt><dd key=${k + '-v'}>${v === null || v === undefined || v === '' ? '\u2013' : v}</dd>`)}
    </dl>`;
  }

  function SidePanel({ mapSnap, state, connected, notice, onCancel }) {
    const map = mapSnap && mapSnap.map;
    const robot = state ? state.robot : {};
    const nav = state ? state.navigation : {};
    const pose = robot.pose;
    const speed = robot.speed;
    const result = nav.last_result;
    return html`
      <aside class="side">
        <h1>Topological Navigation</h1>
        <span class="status"><span class=${'dot' + (connected ? ' ok' : '')}></span>
          ${connected ? 'connected' : 'disconnected \u2013 reconnecting\u2026'}</span>

        <h2>Map</h2>
        <div class="card"><${KV} rows=${[
          ['Name', map && map.name],
          ['Frame', map && map.frame],
          ['Nodes / edges', map ? map.nodes.length + ' / ' + map.edges.length : null],
        ]} /></div>

        <h2>Robot</h2>
        <div class="card"><${KV} rows=${[
          ['Current node', robot.current_node],
          ['Closest node', robot.closest_node],
          ['Position', pose ? 'x ' + fmt(pose.x, 2) + ', y ' + fmt(pose.y, 2) : null],
          ['Heading', pose ? fmt(pose.yaw * 180 / Math.PI, 0, '\u00b0') : null],
          ['Speed', speed ? fmt(speed.linear, 2, 'm/s') : null],
          ['Turn rate', speed ? fmt(speed.angular, 2, 'rad/s') : null],
        ]} /></div>

        <h2>Navigation</h2>
        <div class="card">
          <div style=${{ marginBottom: '6px' }}>
            <span class=${'badge' + (nav.active ? ' active' : '')}>${nav.active ? 'NAVIGATING' : 'IDLE'}</span>
            ${nav.status && html` <span class="badge">${nav.status}</span>`}
          </div>
          <${KV} rows=${[
            ['Destination', nav.destination],
            ['Current goal', nav.current_goal],
            ['Current edge', nav.current_edge],
            ['Last result', result ? result.target + ': ' + result.outcome : null],
          ]} />
          ${nav.route && nav.route.length > 0 && html`
            <div class="route">Route: ${nav.route.join(' \u2192 ')}</div>`}
          <button class="btn danger" disabled=${!nav.active} onClick=${onCancel}>Cancel goal</button>
          ${nav.message && html`<div class="notice">${nav.message}</div>`}
          ${notice && html`<div class=${'notice' + (notice.error ? ' error' : '')}>${notice.text}</div>`}
        </div>

        <h2>Legend</h2>
        <ul class="legend">
          <li><span class="swatch" style=${{ background: COLORS.node }}></span>Node</li>
          <li><span class="swatch" style=${{ background: COLORS.roboflow }}></span>Node with roboflow enabled</li>
          <li><span class="swatch ring" style=${{ borderColor: COLORS.currentNode }}></span>Current node</li>
          <li><span class="swatch ring" style=${{ borderColor: COLORS.destination }}></span>Destination</li>
          <li><span class="swatch ring" style=${{ borderColor: COLORS.currentGoal }}></span>Current goal</li>
          <li><span class="swatch line" style=${{ background: COLORS.route }}></span>Planned route</li>
          <li><span class="swatch line" style=${{ background: COLORS.currentEdge }}></span>Current edge</li>
          <li><span class="swatch" style=${{ background: COLORS.robot }}></span>Robot (arrow = heading)</li>
        </ul>
        <p class="notice">Hover a node for its properties, click it to send the robot there.
          Scroll to zoom, drag to pan.</p>
      </aside>`;
  }

  // ------------------------------------------------------------------
  // App
  // ------------------------------------------------------------------
  function App() {
    const { mapSnap, state, connected } = useServerState();
    const cyRef = useRef(null);
    const [notice, setNotice] = useState(null);
    const [confirmTarget, setConfirmTarget] = useState(null);

    const report = (promise) => promise
      .then((d) => setNotice({ text: d.message, error: false }))
      .catch((err) => setNotice({ text: err.message, error: true }));

    const onNodeClick = (name) => setConfirmTarget(name);
    const confirmGoto = () => {
      report(postJson('api/goto', { node: confirmTarget }));
      setConfirmTarget(null);
    };
    const onCancel = () => report(postJson('api/cancel', {}));

    const fit = () => { if (cyRef.current) cyRef.current.fit(cyRef.current.nodes('.topo'), 40); };
    const centreRobot = () => {
      const cy = cyRef.current;
      if (!cy) return;
      const r = cy.getElementById('__robot');
      if (r.nonempty()) cy.center(r);
    };
    const hasPose = !!(state && state.robot.pose);

    return html`
      <div class="app">
        <div class="map-pane">
          <${MapView} mapSnap=${mapSnap} state=${state} onNodeClick=${onNodeClick} cyRef=${cyRef} />
          <div class="map-toolbar">
            <button class="btn" onClick=${fit}>Fit map</button>
            <button class="btn" onClick=${centreRobot} disabled=${!hasPose}>Centre on robot</button>
          </div>
        </div>
        <${SidePanel} mapSnap=${mapSnap} state=${state} connected=${connected}
          notice=${notice} onCancel=${onCancel} />
        ${confirmTarget && html`<${ConfirmModal}
          message=${'Send the robot to node "' + confirmTarget + '"?'}
          onConfirm=${confirmGoto} onCancel=${() => setConfirmTarget(null)} />`}
      </div>`;
  }

  ReactDOM.createRoot(document.getElementById('root')).render(html`<${App} />`);
}());
