/* Reusable Interactive Zoom / Pan / Fullscreen Expandable Modal for SVG Block Diagrams */
(function () {
  if (window.__diagramZoomInit) return;
  window.__diagramZoomInit = true;

  const css = `
  .fig-diagram { position: relative; }
  .dz-bar {
    display: flex; align-items: center; justify-content: space-between; gap: 8px;
    margin-bottom: 8px; flex-wrap: wrap;
  }
  .dz-label {
    font-size: 11.5px; font-weight: 700; letter-spacing: 0.04em; text-transform: uppercase;
    color: #93a4be; font-family: "SFMono-Regular", Menlo, Consolas, monospace;
  }
  .dz-btns { display: flex; align-items: center; gap: 5px; }
  .dz-btn {
    background: #131a26; color: #dbe7f8; border: 1px solid #2b3b55; border-radius: 7px;
    padding: 4px 9px; font-size: 12px; font-weight: 700; cursor: pointer;
    font-family: "SFMono-Regular", Menlo, Consolas, monospace;
    transition: border-color 0.12s ease, background 0.12s ease;
  }
  .dz-btn:hover { border-color: #38bdf8; background: #192438; }
  .dz-btn.expand { background: rgba(56,189,248,0.14); border-color: rgba(56,189,248,0.45); color: #7dd3fc; }
  .dz-pct {
    min-width: 44px; text-align: center; font-size: 11.5px; color: #94a3b8;
    font-family: "SFMono-Regular", Menlo, Consolas, monospace; font-weight: 700;
  }
  .dz-stage {
    overflow: hidden; border-radius: 9px; background: #090d14;
    border: 1px solid #1e2a3d; cursor: grab; touch-action: none;
  }
  .dz-stage.dragging { cursor: grabbing; }
  .dz-inner {
    transform-origin: 0 0; will-change: transform;
  }
  .dz-inner svg { width: 100%; height: auto; display: block; user-select: none; }

  .dz-modal-bg {
    position: fixed; inset: 0; z-index: 9999;
    background: rgba(5, 9, 16, 0.90); backdrop-filter: blur(5px);
    display: none; flex-direction: column; padding: 14px;
  }
  .dz-modal-bg.open { display: flex; }
  .dz-modal-top {
    display: flex; align-items: center; justify-content: space-between; gap: 10px;
    padding: 8px 12px; background: #111827; border: 1px solid #26334a;
    border-radius: 12px; margin-bottom: 10px; flex-wrap: wrap;
  }
  .dz-modal-title { font-size: 14px; font-weight: 700; color: #f8fafc; }
  .dz-modal-stage {
    flex: 1; overflow: hidden; background: #090d14; border: 1px solid #26334a;
    border-radius: 12px; position: relative; cursor: grab; touch-action: none;
    display: flex; align-items: center; justify-content: center;
  }
  .dz-modal-stage.dragging { cursor: grabbing; }
  .dz-modal-inner { transform-origin: center center; will-change: transform; width: min(94vw, 1320px); }
  .dz-modal-inner svg { width: 100%; height: auto; display: block; user-select: none; }
  .dz-modal-cap {
    margin-top: 8px; text-align: center; font-size: 13px; color: #cbd5e1;
  }
  `;

  const styleEl = document.createElement('style');
  styleEl.textContent = css;
  document.head.appendChild(styleEl);

  function createModal() {
    const bg = document.createElement('div');
    bg.className = 'dz-modal-bg';
    bg.innerHTML = `
      <div class="dz-modal-top">
        <div class="dz-modal-title" id="dzModalTitle">Block Diagram</div>
        <div class="dz-btns">
          <button class="dz-btn" id="dzMOut" title="Zoom Out">&minus;</button>
          <span class="dz-pct" id="dzMPct">100%</span>
          <button class="dz-btn" id="dzMIn" title="Zoom In">+</button>
          <button class="dz-btn" id="dzMRst" title="Reset View">Reset</button>
          <button class="dz-btn expand" id="dzMClose" title="Close (Esc)">&times; Close</button>
        </div>
      </div>
      <div class="dz-modal-stage" id="dzMStage">
        <div class="dz-modal-inner" id="dzMInner"></div>
      </div>
      <div class="dz-modal-cap" id="dzMCap">Drag to pan · Scroll to zoom · Press Esc to close</div>
    `;
    document.body.appendChild(bg);

    let scale = 1, tx = 0, ty = 0, dragging = false, sx = 0, sy = 0;
    const stage = bg.querySelector('#dzMStage');
    const inner = bg.querySelector('#dzMInner');
    const pct = bg.querySelector('#dzMPct');

    function apply() {
      inner.style.transform = `translate(${tx}px, ${ty}px) scale(${scale})`;
      pct.textContent = Math.round(scale * 100) + '%';
    }
    function setScale(s) {
      scale = Math.max(0.5, Math.min(4.5, s));
      if (scale <= 1.01 && Math.abs(tx) < 2 && Math.abs(ty) < 2) { tx = 0; ty = 0; }
      apply();
    }
    function reset() { scale = 1; tx = 0; ty = 0; apply(); }
    function close() { bg.classList.remove('open'); document.body.style.overflow = ''; }

    bg.querySelector('#dzMIn').onclick = () => setScale(scale * 1.25);
    bg.querySelector('#dzMOut').onclick = () => setScale(scale / 1.25);
    bg.querySelector('#dzMRst').onclick = reset;
    bg.querySelector('#dzMClose').onclick = close;
    bg.addEventListener('click', (e) => { if (e.target === bg) close(); });
    window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && bg.classList.contains('open')) close(); });

    stage.addEventListener('wheel', (e) => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
      setScale(scale * factor);
    }, { passive: false });

    stage.addEventListener('pointerdown', (e) => {
      dragging = true; sx = e.clientX - tx; sy = e.clientY - ty;
      stage.classList.add('dragging'); stage.setPointerCapture(e.pointerId);
    });
    stage.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      tx = e.clientX - sx; ty = e.clientY - sy; apply();
    });
    const endDrag = () => { dragging = false; stage.classList.remove('dragging'); };
    stage.addEventListener('pointerup', endDrag);
    stage.addEventListener('pointercancel', endDrag);

    return {
      open(svgEl, title, capHtml) {
        inner.innerHTML = '';
        inner.appendChild(svgEl.cloneNode(true));
        bg.querySelector('#dzModalTitle').textContent = title || 'Interactive Block Diagram';
        bg.querySelector('#dzMCap').innerHTML = (capHtml ? capHtml + ' &nbsp;&middot;&nbsp; ' : '') + 'Drag to pan · Scroll to zoom · Esc to close';
        reset();
        bg.classList.add('open');
        document.body.style.overflow = 'hidden';
      }
    };
  }

  function init() {
    const modal = createModal();
    document.querySelectorAll('.fig-diagram').forEach((fig) => {
      const svg = fig.querySelector('svg');
      if (!svg) return;
      const title = fig.getAttribute('data-title') || 'Block Diagram';
      const capEl = fig.querySelector('.cap');
      const capHtml = capEl ? capEl.innerHTML : '';

      const bar = document.createElement('div');
      bar.className = 'dz-bar';
      bar.innerHTML = `
        <span class="dz-label">${title} · Double-click or Expand to zoom</span>
        <div class="dz-btns">
          <button type="button" class="dz-btn dz-out" title="Zoom Out">&minus;</button>
          <span class="dz-pct">100%</span>
          <button type="button" class="dz-btn dz-in" title="Zoom In">+</button>
          <button type="button" class="dz-btn dz-rst" title="Reset">Reset</button>
          <button type="button" class="dz-btn expand dz-exp" title="Fullscreen Zoom">&#x26F6; Expand</button>
        </div>
      `;

      const stage = document.createElement('div');
      stage.className = 'dz-stage';
      const inner = document.createElement('div');
      inner.className = 'dz-inner';
      svg.parentNode.insertBefore(bar, svg);
      svg.parentNode.insertBefore(stage, svg);
      inner.appendChild(svg);
      stage.appendChild(inner);

      let scale = 1, tx = 0, ty = 0, dragging = false, sx = 0, sy = 0;
      const pct = bar.querySelector('.dz-pct');
      function apply() {
        inner.style.transform = `translate(${tx}px, ${ty}px) scale(${scale})`;
        pct.textContent = Math.round(scale * 100) + '%';
      }
      function setScale(s) {
        scale = Math.max(0.75, Math.min(3.5, s));
        if (scale <= 1.01) { tx = 0; ty = 0; }
        apply();
      }
      function reset() { scale = 1; tx = 0; ty = 0; apply(); }

      bar.querySelector('.dz-in').onclick = () => setScale(scale * 1.25);
      bar.querySelector('.dz-out').onclick = () => setScale(scale / 1.25);
      bar.querySelector('.dz-rst').onclick = reset;
      bar.querySelector('.dz-exp').onclick = () => modal.open(svg, title, capHtml);
      stage.addEventListener('dblclick', () => modal.open(svg, title, capHtml));

      stage.addEventListener('pointerdown', (e) => {
        if (scale <= 1.01) return;
        dragging = true; sx = e.clientX - tx; sy = e.clientY - ty;
        stage.classList.add('dragging'); stage.setPointerCapture(e.pointerId);
      });
      stage.addEventListener('pointermove', (e) => {
        if (!dragging) return;
        tx = e.clientX - sx; ty = e.clientY - sy; apply();
      });
      const endDrag = () => { dragging = false; stage.classList.remove('dragging'); };
      stage.addEventListener('pointerup', endDrag);
      stage.addEventListener('pointercancel', endDrag);
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
