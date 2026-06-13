const char INDEX_HTML[] PROGMEM = R"DIGGER(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Malaki SuperTracks</title>
<style>
/* Web fonts omitted on purpose: the digger AP has no internet, so an @import
   here only stalls first paint while the browser waits for a fetch that can
   never succeed. System fallbacks (below) render instantly. */
*{margin:0;padding:0;box-sizing:border-box}

body{background:#0a0b0f;color:#fff;font-family:'Rajdhani',sans-serif;overflow:hidden;
  height:100vh;width:100vw;display:flex;flex-direction:column}

/* Metallic dark panel background via CSS gradient */
.dash-frame{flex:1;position:relative;
  background:
    radial-gradient(ellipse at 50% 0%, rgba(60,50,30,0.15) 0%, transparent 50%),
    linear-gradient(180deg, #111214 0%, #0d0e10 30%, #0a0b0d 100%);
  background-size:100% 100%;
  border:2px solid #1a1a1a;
  box-shadow:inset 0 1px 0 rgba(255,255,255,0.05), inset 0 -1px 0 rgba(0,0,0,0.5)}

/* Top bar */
.top-bar{display:flex;justify-content:space-between;align-items:center;
  padding:4px 16px;background:linear-gradient(180deg,#181a1f,#111214);
  border-bottom:1px solid #2a2520}
.top-label{font-family:'Orbitron';font-size:0.55em;letter-spacing:3px;color:#554}
.top-status{display:flex;align-items:center;gap:6px}
.dot-live{width:6px;height:6px;border-radius:50%;background:#0f0;box-shadow:0 0 6px #0f0;
  animation:blink 2s infinite}
@keyframes blink{50%{opacity:0.3}}

/* Title banner */
.title-bar{text-align:center;padding:6px 0 2px;position:relative}
.title-main{font-family:'Orbitron';font-size:1.8em;font-weight:900;letter-spacing:4px;
  color:#f0a000;text-shadow:0 0 20px rgba(240,160,0,0.4),0 2px 4px rgba(0,0,0,0.8)}
.title-sub{font-family:'Orbitron';font-size:0.6em;letter-spacing:6px;color:#665;margin-top:-4px}
.title-bar::after{content:'';display:block;height:2px;margin-top:4px;
  background:linear-gradient(90deg,transparent 5%,#3a2a10 20%,#f0a000 50%,#3a2a10 80%,transparent 95%)}

/* Main grid */
.main{flex:1;display:grid;grid-template-columns:1fr auto 1fr;gap:0;padding:4px 8px;
  align-items:start;min-height:0}

/* Track panels */
.track-panel{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:4px}
.track-header{grid-column:1/-1;text-align:center;padding:2px 0}
.track-title{font-family:'Orbitron';font-size:0.7em;font-weight:700;letter-spacing:4px}
.track-left .track-title{color:#00ccff}
.track-right .track-title{color:#ff8800}
.track-header::after{content:'';display:block;height:1px;margin-top:2px;
  background:linear-gradient(90deg,transparent,currentColor,transparent)}
.track-left .track-header::after{color:#00ccff33}
.track-right .track-header::after{color:#ff880033}

/* Round gauge */
.gauge-cell{display:flex;flex-direction:column;align-items:center}
.gauge-wrap{position:relative;width:120px;height:120px}
.gauge-bg{position:absolute;inset:0;border-radius:50%;
  background:radial-gradient(circle at 40% 35%,#1a1c22,#0e0f12 60%,#090a0c);
  border:2px solid #222;box-shadow:inset 0 2px 8px rgba(0,0,0,0.8),0 2px 8px rgba(0,0,0,0.5)}
.gauge-svg{position:absolute;inset:0;width:100%;height:100%}
.g-tick{stroke:#333;stroke-width:1.5;stroke-linecap:round}
.g-tick-major{stroke:#555;stroke-width:2}
.g-arc{fill:none;stroke-width:6;stroke-linecap:round;opacity:0.7}
.g-needle{fill:#f44;filter:drop-shadow(0 0 3px rgba(255,60,60,0.6));
  transform-origin:50px 50px;transition:transform 0.15s ease-out}
.g-cap{fill:radial-gradient(circle,#333,#111);stroke:#444;stroke-width:1}
.gauge-val{position:absolute;bottom:18px;left:50%;transform:translateX(-50%);
  font-family:'Orbitron';font-size:1.3em;font-weight:700;text-shadow:0 0 10px currentColor}
.gauge-label{font-family:'Orbitron';font-size:0.5em;letter-spacing:2px;color:#555;margin-top:2px}

/* Center column */
.center-col{display:flex;flex-direction:column;align-items:center;justify-content:space-between;
  padding:0 8px;min-width:160px}

/* Direction arrows */
.dir-section{display:flex;gap:20px;align-items:center;margin:4px 0}
.dir-col{text-align:center}
.dir-arrows{font-size:1.4em;line-height:1;transition:color 0.3s,text-shadow 0.3s}
.dir-arrows.fwd{color:#00ccff;text-shadow:0 0 10px rgba(0,200,255,0.5)}
.dir-arrows.rev{color:#ff4444;text-shadow:0 0 10px rgba(255,60,60,0.5)}
.dir-arrows.off{color:#222;text-shadow:none}
.dir-label{font-family:'Orbitron';font-size:0.5em;letter-spacing:2px;color:#555}

/* Excavator silhouette */
.excavator-icon{width:100px;height:80px;margin:4px 0;opacity:0.7}

/* Vehicle direction arrow */
.vehicle-dir{font-size:2em;transition:color 0.3s,transform 0.3s,text-shadow 0.3s}
.vehicle-dir.fwd{color:#f0a000;text-shadow:0 0 15px rgba(240,160,0,0.5)}
.vehicle-dir.rev{color:#f44;transform:rotate(180deg);text-shadow:0 0 15px rgba(255,60,60,0.5)}
.vd-label{font-family:'Orbitron';font-size:0.5em;letter-spacing:2px;color:#664}

/* Bottom bar */
.bottom-bar{display:grid;grid-template-columns:1fr 1fr 1fr 1fr 1fr;gap:6px;
  padding:4px 8px;background:linear-gradient(180deg,#111214,#0d0e10);
  border-top:1px solid #2a2520}
.bot-cell{text-align:center;padding:4px;border-radius:4px;
  background:rgba(255,255,255,0.02);border:1px solid #1a1a1a}
.bot-val{font-family:'Orbitron';font-size:1.1em;font-weight:700}
.bot-label{font-family:'Orbitron';font-size:0.45em;letter-spacing:2px;color:#555;margin-top:1px}

/* Battery */
.bat-gauge{width:60px;height:28px;border:2px solid #444;border-radius:3px;
  position:relative;display:inline-block;vertical-align:middle}
.bat-gauge::after{content:'';position:absolute;right:-5px;top:7px;width:4px;height:10px;
  background:#444;border-radius:0 2px 2px 0}
.bat-fill{position:absolute;inset:2px;border-radius:1px;transition:width 0.5s ease}

/* Mode pills */
.mode-bar{display:flex;justify-content:center;gap:6px;padding:3px 0}
.mode-pill{padding:3px 12px;font-family:'Orbitron';font-size:0.5em;font-weight:700;
  letter-spacing:2px;background:rgba(255,255,255,0.02);color:#333;
  border:1px solid #222;border-radius:2px;transition:all 0.3s}
.mode-pill.on-rc{background:rgba(0,200,255,0.12);color:#0cf;border-color:#0cf;box-shadow:0 0 10px rgba(0,200,255,0.15)}
.mode-pill.on-joy{background:rgba(255,140,0,0.12);color:#f80;border-color:#f80;box-shadow:0 0 10px rgba(255,140,0,0.15)}
.mode-pill.on-blend{background:rgba(180,100,255,0.12);color:#b8f;border-color:#b8f;box-shadow:0 0 10px rgba(180,100,255,0.15)}

/* Speed gauge in bottom */
.speed-big{font-family:'Orbitron';font-size:1.6em;font-weight:900;color:#fff;
  text-shadow:0 0 15px rgba(255,255,255,0.3)}

/* Gear badge */
.gear-badge{display:inline-block;padding:2px 10px;font-family:'Orbitron';font-size:0.55em;
  font-weight:700;letter-spacing:2px;border:1px solid;border-radius:2px;margin-top:1px}
.g-eco{color:#0c6;border-color:#0c6;background:rgba(0,200,100,0.08)}
.g-norm{color:#fc0;border-color:#fc0;background:rgba(255,200,0,0.08)}
.g-turbo{color:#f44;border-color:#f44;background:rgba(255,60,60,0.08);animation:tp .5s infinite}
@keyframes tp{50%{box-shadow:0 0 12px rgba(255,60,60,0.4)}}

/* Responsive */
@media(max-width:700px){
  .main{grid-template-columns:1fr 1fr;grid-template-rows:auto auto}
  .center-col{grid-column:1/-1;grid-row:1;flex-direction:row;justify-content:center;gap:20px}
  .gauge-wrap{width:90px;height:90px}
  .bottom-bar{grid-template-columns:1fr 1fr 1fr}
}
</style>
</head>
<body>
<div class="dash-frame">

<div class="top-bar">
  <span class="top-label">OPERATOR SYSTEM</span>
  <span class="top-label">LIVE TELEMETRY</span>
  <div class="top-status"><span class="dot-live"></span><span class="top-label" id="conn">10 Hz</span></div>
</div>

<div class="title-bar">
  <div class="title-main">MALAKI SUPERTRACKS</div>
  <div class="title-sub">EXCAVATOR COMMAND</div>
</div>

<div class="mode-bar">
  <div class="mode-pill" id="mRC">RC CONTROL</div>
  <div class="mode-pill" id="mJoy">JOYSTICK</div>
  <div class="mode-pill" id="mBlend">DUAL MIX</div>
</div>

<div class="main">

  <!-- LEFT TRACK -->
  <div class="track-panel track-left">
    <div class="track-header"><div class="track-title">LEFT TRACK</div></div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <defs><filter id="gl"><feGaussianBlur stdDeviation="2" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter></defs>
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad1)" opacity="0.4"/>
          <line x1="50" y1="18" x2="50" y2="24" class="g-tick-major"/>
          <line x1="82" y1="50" x2="76" y2="50" class="g-tick-major"/>
          <line x1="50" y1="82" x2="50" y2="76" class="g-tick"/>
          <line x1="18" y1="50" x2="24" y2="50" class="g-tick-major"/>
          <polygon id="ndlEL" points="49,22 51,22 50.5,52 49.5,52" fill="#0cf" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="5" fill="#1a1c22" stroke="#333" stroke-width="1"/>
        </svg>
        <div class="gauge-val" id="vEL" style="color:#0cf">41°</div>
      </div>
      <div class="gauge-label">ESC TEMP</div>
    </div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad1)" opacity="0.4"/>
          <line x1="50" y1="18" x2="50" y2="24" class="g-tick-major"/><line x1="82" y1="50" x2="76" y2="50" class="g-tick-major"/>
          <line x1="18" y1="50" x2="24" y2="50" class="g-tick-major"/><line x1="50" y1="82" x2="50" y2="76" class="g-tick"/>
          <polygon id="ndlML" points="49,22 51,22 50.5,52 49.5,52" fill="#0cf" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="5" fill="#1a1c22" stroke="#333" stroke-width="1"/>
        </svg>
        <div class="gauge-val" id="vML" style="color:#0cf">47°</div>
      </div>
      <div class="gauge-label">MOTOR TEMP</div>
    </div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad2)" opacity="0.5"/>
          <line x1="50" y1="18" x2="50" y2="22" class="g-tick-major"/><line x1="82" y1="50" x2="78" y2="50" class="g-tick-major"/>
          <line x1="18" y1="50" x2="22" y2="50" class="g-tick-major"/><line x1="50" y1="82" x2="50" y2="78" class="g-tick"/>
          <line x1="27" y1="27" x2="30" y2="30" class="g-tick"/><line x1="73" y1="27" x2="70" y2="30" class="g-tick"/>
          <polygon id="ndlRL" points="49,20 51,20 50.5,52 49.5,52" fill="#ff4444" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="6" fill="#1a1c22" stroke="#444" stroke-width="1.5"/>
        </svg>
        <div class="gauge-val" id="vRL" style="color:#0cf">1280</div>
      </div>
      <div class="gauge-label">RPM</div>
    </div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad3)" opacity="0.5"/>
          <line x1="50" y1="18" x2="50" y2="22" class="g-tick-major"/><line x1="82" y1="50" x2="78" y2="50" class="g-tick-major"/>
          <line x1="18" y1="50" x2="22" y2="50" class="g-tick-major"/><line x1="50" y1="82" x2="50" y2="78" class="g-tick"/>
          <polygon id="ndlCL" points="49,22 51,22 50.5,52 49.5,52" fill="#0cf" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="5" fill="#1a1c22" stroke="#333" stroke-width="1"/>
        </svg>
        <div class="gauge-val" id="vCL" style="color:#0cf">22A</div>
      </div>
      <div class="gauge-label">CURRENT</div>
    </div>
  </div>

  <!-- CENTER -->
  <div class="center-col">
    <div class="dir-section">
      <div class="dir-col">
        <div class="dir-label">L</div>
        <div class="dir-arrows fwd" id="dirL">&#x25B2;&#x25B2;</div>
      </div>
      <div style="text-align:center">
        <!-- Excavator SVG silhouette -->
        <svg class="excavator-icon" viewBox="0 0 100 80" fill="none">
          <rect x="15" y="55" width="70" height="18" rx="4" fill="#1a1a1a" stroke="#f80" stroke-width="1"/>
          <rect x="10" y="60" width="80" height="10" rx="5" fill="none" stroke="#333" stroke-width="2" stroke-dasharray="4 3"/>
          <rect x="30" y="25" width="40" height="30" rx="3" fill="#1a1a1a" stroke="#f80" stroke-width="1"/>
          <rect x="34" y="29" width="18" height="12" rx="1" fill="none" stroke="#0cf" stroke-width="0.8" opacity="0.5"/>
          <line x1="70" y1="35" x2="85" y2="15" stroke="#f80" stroke-width="2.5" stroke-linecap="round"/>
          <line x1="85" y1="15" x2="92" y2="30" stroke="#f80" stroke-width="2" stroke-linecap="round"/>
          <line x1="92" y1="30" x2="95" y2="45" stroke="#f80" stroke-width="1.5" stroke-linecap="round"/>
          <circle cx="20" cy="65" r="6" fill="none" stroke="#333" stroke-width="1.5"/>
          <circle cx="80" cy="65" r="6" fill="none" stroke="#333" stroke-width="1.5"/>
        </svg>
      </div>
      <div class="dir-col">
        <div class="dir-label">R</div>
        <div class="dir-arrows fwd" id="dirR">&#x25B2;&#x25B2;</div>
      </div>
    </div>

    <div style="text-align:center">
      <div class="vehicle-dir fwd" id="vDir">&#x25B2;</div>
      <div class="vd-label">DIRECTION</div>
    </div>

    <div style="text-align:center;margin:4px 0">
      <div class="speed-big" id="spd">0.0</div>
      <div style="font-family:Orbitron;font-size:0.55em;color:#554;letter-spacing:3px">KM/H</div>
      <div class="gear-badge g-norm" id="gearBadge">NORMAL</div>
    </div>

    <div style="text-align:center">
      <div style="display:flex;gap:10px;justify-content:center;align-items:center">
        <div>
          <div class="bat-gauge"><div class="bat-fill" id="batLFill" style="width:80%;background:linear-gradient(90deg,#0c6,#4f8)"></div></div>
          <div style="font-family:Orbitron;font-size:0.5em;color:#555;margin-top:1px">L <span id="batLV">11.4V</span></div>
        </div>
        <div style="font-family:Orbitron;font-size:1.4em;font-weight:900;color:#0c6" id="batPct">82%</div>
        <div>
          <div class="bat-gauge"><div class="bat-fill" id="batRFill" style="width:78%;background:linear-gradient(90deg,#0c6,#4f8)"></div></div>
          <div style="font-family:Orbitron;font-size:0.5em;color:#555;margin-top:1px">R <span id="batRV">11.2V</span></div>
        </div>
      </div>
      <div style="font-family:Orbitron;font-size:0.45em;color:#553;letter-spacing:2px;margin-top:2px">BATTERY</div>
    </div>
  </div>

  <!-- RIGHT TRACK (mirrored) -->
  <div class="track-panel track-right">
    <div class="track-header"><div class="track-title">RIGHT TRACK</div></div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad1)" opacity="0.4"/>
          <line x1="50" y1="18" x2="50" y2="24" class="g-tick-major"/><line x1="82" y1="50" x2="76" y2="50" class="g-tick-major"/>
          <line x1="18" y1="50" x2="24" y2="50" class="g-tick-major"/><line x1="50" y1="82" x2="50" y2="76" class="g-tick"/>
          <polygon id="ndlER" points="49,22 51,22 50.5,52 49.5,52" fill="#f80" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="5" fill="#1a1c22" stroke="#333" stroke-width="1"/>
        </svg>
        <div class="gauge-val" id="vER" style="color:#f80">43°</div>
      </div>
      <div class="gauge-label">ESC TEMP</div>
    </div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad1)" opacity="0.4"/>
          <line x1="50" y1="18" x2="50" y2="24" class="g-tick-major"/><line x1="82" y1="50" x2="76" y2="50" class="g-tick-major"/>
          <line x1="18" y1="50" x2="24" y2="50" class="g-tick-major"/><line x1="50" y1="82" x2="50" y2="76" class="g-tick"/>
          <polygon id="ndlMR" points="49,22 51,22 50.5,52 49.5,52" fill="#f80" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="5" fill="#1a1c22" stroke="#333" stroke-width="1"/>
        </svg>
        <div class="gauge-val" id="vMR" style="color:#f80">49°</div>
      </div>
      <div class="gauge-label">MOTOR TEMP</div>
    </div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad2)" opacity="0.5"/>
          <line x1="50" y1="18" x2="50" y2="22" class="g-tick-major"/><line x1="82" y1="50" x2="78" y2="50" class="g-tick-major"/>
          <line x1="18" y1="50" x2="22" y2="50" class="g-tick-major"/><line x1="50" y1="82" x2="50" y2="78" class="g-tick"/>
          <line x1="27" y1="27" x2="30" y2="30" class="g-tick"/><line x1="73" y1="27" x2="70" y2="30" class="g-tick"/>
          <polygon id="ndlRR" points="49,20 51,20 50.5,52 49.5,52" fill="#ff4444" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="6" fill="#1a1c22" stroke="#444" stroke-width="1.5"/>
        </svg>
        <div class="gauge-val" id="vRR" style="color:#f80">1315</div>
      </div>
      <div class="gauge-label">RPM</div>
    </div>

    <div class="gauge-cell">
      <div class="gauge-wrap">
        <div class="gauge-bg"></div>
        <svg class="gauge-svg" viewBox="0 0 100 100">
          <circle cx="50" cy="50" r="38" fill="none" stroke="#1a1a1a" stroke-width="8"/>
          <path d="M 20.4 79.6 A 38 38 0 1 1 79.6 79.6" fill="none" stroke-width="6" stroke="url(#arcGrad3)" opacity="0.5"/>
          <line x1="50" y1="18" x2="50" y2="22" class="g-tick-major"/><line x1="82" y1="50" x2="78" y2="50" class="g-tick-major"/>
          <line x1="18" y1="50" x2="22" y2="50" class="g-tick-major"/><line x1="50" y1="82" x2="50" y2="78" class="g-tick"/>
          <polygon id="ndlCR" points="49,22 51,22 50.5,52 49.5,52" fill="#f80" filter="url(#gl)" transform="rotate(0,50,50)"/>
          <circle cx="50" cy="50" r="5" fill="#1a1c22" stroke="#333" stroke-width="1"/>
        </svg>
        <div class="gauge-val" id="vCR" style="color:#f80">24A</div>
      </div>
      <div class="gauge-label">CURRENT</div>
    </div>
  </div>

</div>

<div class="bottom-bar">
  <div class="bot-cell"><div class="bot-val" id="thrL" style="color:#0cf">65%</div><div class="bot-label">THROTTLE L</div></div>
  <div class="bot-cell"><div class="bot-val" id="curTot" style="color:#f80">46A</div><div class="bot-label">TOTAL AMPS</div></div>
  <div class="bot-cell"><div class="bot-val" id="pwrTot" style="color:#fc0">520W</div><div class="bot-label">POWER</div></div>
  <div class="bot-cell"><div class="bot-val" id="curTotR" style="color:#f80">--</div><div class="bot-label">UPTIME</div></div>
  <div class="bot-cell"><div class="bot-val" id="thrR" style="color:#c8f">68%</div><div class="bot-label">THROTTLE R</div></div>
</div>

</div>

<!-- Shared SVG gradients -->
<svg style="position:absolute;width:0;height:0">
  <defs>
    <linearGradient id="arcGrad1" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0%" stop-color="#00cc44"/><stop offset="50%" stop-color="#ffcc00"/><stop offset="100%" stop-color="#ff4400"/>
    </linearGradient>
    <linearGradient id="arcGrad2" x1="0" y1="0" x2="1" y2="0">
      <stop offset="0%" stop-color="#0066ff"/><stop offset="40%" stop-color="#00ccff"/><stop offset="70%" stop-color="#ffcc00"/><stop offset="100%" stop-color="#ff4400"/>
    </linearGradient>
    <linearGradient id="arcGrad3" x1="0" y1="0" x2="1" y2="0">
      <stop offset="0%" stop-color="#00ccff"/><stop offset="60%" stop-color="#ffcc00"/><stop offset="100%" stop-color="#ff4400"/>
    </linearGradient>
  </defs>
</svg>

<script>
// ── Live telemetry client ───────────────────────────────────────────
// Polls the Arduino's /data JSON endpoint and renders both tracks.
// When the page is served BY the Arduino (http://192.168.4.1/) it uses a
// relative URL; when opened as a local file it targets the AP IP directly.
const AP_IP = '192.168.4.1';
const SAME_ORIGIN = (location.protocol === 'http:' && location.host);
const EVENTS_URL = SAME_ORIGIN ? '/events' : 'http://' + AP_IP + '/events';
const STALE_MS = 2000;        // no data this long → NO LINK

const MR=6000, GR=20, WC=0.95, a=0.30;   // RPM full-scale, gear ratio, wheel circ., needle smoothing
const gn=['ECO','NORMAL','TURBO'], gcl=['g-eco','g-norm','g-turbo'];

let eL={r:0,c:0,b:11.4,tM:25,tE:25}, eR={r:0,c:0,b:11.2,tM:25,tE:25};
let lastRx=0, uptime0=0;

function em(p,n){return p+a*(n-p)}
function bp(v){return Math.max(0,Math.min(100,((v-9)/(12.6-9))*100))}
function bc(p){return p>50?'linear-gradient(90deg,#0c6,#4f8)':p>25?'linear-gradient(90deg,#fa0,#fc0)':'linear-gradient(90deg,#f44,#f66)'}
function ndl(id,val,max){const e=document.getElementById(id);
  if(e) e.setAttribute('transform',`rotate(${-135+Math.max(0,Math.min(val/max,1))*270},50,50)`);}
function tc(t){return t<55?'#0c6':t<80?'#fc0':'#f44'}
function set(id,txt){const e=document.getElementById(id); if(e) e.textContent=txt;}

function render(){
  ndl('ndlEL',eL.tE,120);ndl('ndlML',eL.tM,120);ndl('ndlRL',eL.r,MR);ndl('ndlCL',eL.c,50);
  ndl('ndlER',eR.tE,120);ndl('ndlMR',eR.tM,120);ndl('ndlRR',eR.r,MR);ndl('ndlCR',eR.c,50);

  set('vEL',eL.tE.toFixed(0)+'°');set('vML',eL.tM.toFixed(0)+'°');
  set('vRL',Math.round(eL.r));set('vCL',eL.c.toFixed(1)+'A');
  set('vER',eR.tE.toFixed(0)+'°');set('vMR',eR.tM.toFixed(0)+'°');
  set('vRR',Math.round(eR.r));set('vCR',eR.c.toFixed(1)+'A');
  [['vEL',eL.tE],['vML',eL.tM],['vER',eR.tE],['vMR',eR.tM]].forEach(([id,t])=>{
    const e=document.getElementById(id); if(e) e.style.color=tc(t);});

  set('spd',((eL.r+eR.r)/2/GR*WC*60/1000).toFixed(1));

  const bL=bp(eL.b),bR=bp(eR.b),bMin=Math.min(bL,bR);
  const fL=document.getElementById('batLFill'),fR=document.getElementById('batRFill');
  if(fL){fL.style.width=bL+'%';fL.style.background=bc(bL);}
  if(fR){fR.style.width=bR+'%';fR.style.background=bc(bR);}
  set('batLV',eL.b.toFixed(1)+'V');set('batRV',eR.b.toFixed(1)+'V');
  set('batPct',bMin.toFixed(0)+'%');
  const bpEl=document.getElementById('batPct'); if(bpEl) bpEl.style.color=bMin>50?'#0c6':bMin>25?'#fc0':'#f44';
}

function dirArrow(out){return out>1530?1:(out<1470?-1:0);}
function applyDir(outL,outR){
  [['dirL',dirArrow(outL)],['dirR',dirArrow(outR)]].forEach(([id,d])=>{
    const e=document.getElementById(id); if(!e) return;
    e.className='dir-arrows '+(d>0?'fwd':d<0?'rev':'off');
    e.innerHTML=d<0?'&#9660;&#9660;':'&#9650;&#9650;';});
  const vd=document.getElementById('vDir'), s=dirArrow(outL)+dirArrow(outR);
  if(vd){vd.className='vehicle-dir '+(s<0?'rev':'fwd');vd.innerHTML=s<0?'&#9660;':'&#9650;';}
}

function applyData(d){
  const s=v=>v/10;
  // Render values directly — the firmware already EMA-smooths voltage/current/
  // temps and sends RPM raw, so a second client-side EMA only adds lag.
  if(d.e0.ok){eL.r=d.e0.rpm;eL.c=s(d.e0.cur);eL.b=s(d.e0.v);eL.tE=d.e0.tE;eL.tM=d.e0.tM;}
  if(d.e1.ok){eR.r=d.e1.rpm;eR.c=s(d.e1.cur);eR.b=s(d.e1.v);eR.tE=d.e1.tE;eR.tM=d.e1.tM;}

  const gb=document.getElementById('gearBadge');
  if(gb){gb.textContent=gn[d.gear]||'--';gb.className='gear-badge '+(gcl[d.gear]||'g-norm');}

  const mp=document.getElementById('mRC'); if(mp) mp.className='mode-pill'+(d.mode===0?' on-rc':'');
  const mj=document.getElementById('mJoy'); if(mj) mj.className='mode-pill'+(d.mode===1?' on-joy':'');
  const mb=document.getElementById('mBlend'); if(mb) mb.className='mode-pill'+(d.mode===2?' on-blend':'');

  applyDir(d.outL,d.outR);
  set('thrL',Math.round((d.outL-1500)/5)+'%');
  set('thrR',Math.round((d.outR-1500)/5)+'%');
  set('curTot',(eL.c+eR.c).toFixed(1)+'A');
  set('pwrTot',Math.round(eL.c*eL.b+eR.c*eR.b)+'W');

  // Per-ESC freshness reflected on the panels (dim a stale side).
  const pL=document.querySelector('.track-left'), pR=document.querySelector('.track-right');
  if(pL) pL.style.opacity=d.e0.ok?'1':'0.35';
  if(pR) pR.style.opacity=d.e1.ok?'1':'0.35';

  render();
}

function setConn(ok){
  set('conn', ok?'LIVE':'NO LINK');
  const dot=document.querySelector('.dot-live');
  if(dot){dot.style.background=ok?'#0f0':'#f44';dot.style.boxShadow='0 0 6px '+(ok?'#0f0':'#f44');}
}

// One persistent SSE stream (auto-reconnects on a dropout — good for the
// breadboard). Far faster than re-opening a connection every poll.
function startStream(){
  set('conn','CONNECTING');
  let es;
  try{ es = new EventSource(EVENTS_URL); }
  catch(e){ setConn(false); return; }
  es.onmessage = (ev)=>{
    try{
      const d = JSON.parse(ev.data);
      lastRx = Date.now(); if(!uptime0) uptime0 = lastRx;
      applyData(d); setConn(true);
      const up = Math.floor((Date.now()-uptime0)/1000);
      set('curTotR', Math.floor(up/60)+':'+String(up%60).padStart(2,'0'));
    }catch(e){}
  };
  es.onerror = ()=>{ setConn(false); };   // browser retries the stream automatically
}
startStream();
setInterval(()=>{ if(Date.now()-lastRx>STALE_MS) setConn(false); }, 500);
</script>
</body>
</html>

)DIGGER";
