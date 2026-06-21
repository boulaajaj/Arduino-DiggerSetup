const char INDEX_HTML[] PROGMEM = R"DIGGER(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, viewport-fit=cover">
<title>Malaki SuperTracks</title>
<!-- Installable home-screen app: launches standalone (no browser chrome) on iOS/iPadOS. -->
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="Digger">
<meta name="theme-color" content="#0a0b0d">
<link rel="apple-touch-icon" href="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAALQAAAC0CAYAAAA9zQYyAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAAZUSURBVHhe7d2vjx1VGMbxbQulQNulDSEIECQEUUMCpgIBAgSihhBMDSGYChKCAEEChhAECBKCqGmCQuJwOBwOh8Ph+A8ueXbZdPede/fO3HNm7nmf8xUf0+6Pe+d+xczsO+ccXD+8uQJcHMR/ADIjaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFghaFgh6IW8+PxTg39DfQS9gOeevbH687uLq1svEPXcCHoBP3z42OrfBwerXz59dPB/qIugZ/b6y9ePYj5x942rg69BPQQ9s9+/unQm6L++v7h65ukbg69DHQQ9o4/vPHEm5hPfvn9l8LWog6Bnorsa/9y/MIj5xO1bh4PvQTmCnsnPn1weRHyaTkXi96AcQc/g3deuDgJeR6ck8XtRhqAr0wWf7jnHeNf5+8cL/MGlMoKu7Ou7Vwbhnuenjy4PfgZ2R9AV6ULvvAvBTe7cvjb4WdgNQVf025ePDGId449vLnFvuhKCruSDN58chDrFF+89PviZmI6gK9CFnf4CGCOdQqcqr7zEvelSBF3B/XvHw0elfv2c4aVSBF0oDh+VYnipDEEX0IVcHD4qpVMXzU/H34VxCLrAZ++sHz4qpfnp+LswDkHvSE+f7HLPeSydysTfie0Iekfbho9KMby0G4Lewdjho1IML01H0BPpQrD0nvNYOqVheGkagp5IT5vE8OakU5v4GrAZQU+g4aMY3BIYXhqPoCfYdfiolOarGV4ah6BHuvd22fBRKc1Zx9eEIYIeQRdmerokRrYkXSDyYO12BD1CreGjUgwvbUfQW7z16rVBWPukuev4GvEQQZ9DF2J6miRGtU+6B8696c0I+hx6iiQG1QKdAsXXimMEvcHcw0elGF5aj6A30NK3MaKWaHiJe9NDBL2GnhqJAbVI89jxtfeOoAM9LbLU8FEphpeGCDpYevioFMNLZxH0KfsaPiql+ez4XnpF0KfUfuB1KewK8BBB/2/TavtZsCvAMYJuZPioFMNLxwj68ObRkrYxkIw0rx3fW2+6D1pPg8QwMtPcdnyPPek66Cmr7WfR+64AXQfd6vBRqZ6Hl7oNWkvXtjx8VEpz3PE996DboPX0R4zASa+7AnQZdOlq+1n0OLzUXdCZho9K6ZRKc93xGDjrLmgtVRs/eGea647HwFlXQddebT+LnnYF6CrorMNHpXoaXuom6OzDR6V6GV7qImj95cz5nvNYPQwvdRH03KvtZ9HDrgD2QS+12n4W7rsCWAftOHxUyn14yTpoLUEbP1AcHM1/x2PlwjZoXQBxIbiZ664AtkHva7X9LFyHlyyD7mX4qJTmweOxy84uaF3w9DJ8VEqnZJoLj8cwM7ugW1ltPwu3XQGsgu51+KiU0/CSTdC6wOl1+KiUTtE0Jx6PaUY2QevpjPhBYTzNicdjmpFF0K2vtp+Fw64AFkEzfFSHw/BS+qAZPqor+/BS6qB1Icg957qy7wqQOuhsq+1nkXlXgLRBZ11tP4usw0tpg2b4aF6aI884vJQyaC0ZGz8A1Kd58njsW5cuaIfV9rPIuCtAuqAZPlpWtuGlVEFridh4wDE/zZfHz6JVaYLWBYqesogHG/PTvf4s96bTBO262n4WWXYFSBE0w0dtyDC8lCJoLQkbDy6Wp+Gl1u9NNx+0nqaIBxb70/quAE0H3dNq+1m0PrzUdNAMH7Wp5eGlZoNm+KhtmkOPn1kLmg2aB17b1uquAE0G3ftq+1m0uCtAc0EzfJRHi8NLzQWtpV7jgUO7NJceP8N9aipoPSURDxjap/n0+FnuSzNBs9p+Xi3tCtBM0Awf5dbK8FITQWtJV4aP8tO8evxsl9ZE0HoqIh4c5NPCrgB7D1rnXjrdgId9L6C+96CBmggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVggaVv4DWreiLpysy+4AAAAASUVORK5CYII=">
<style>
/* Web fonts omitted on purpose: the digger AP has no internet, so an @import
   here only stalls first paint while the browser waits for a fetch that can
   never succeed. System fallbacks (below) render instantly. */
*{margin:0;padding:0;box-sizing:border-box}

/* Fixed-ratio dashboard: 1280x720 stage scaled to fit viewport (letterbox in
   portrait, fill in landscape). NOT responsive — internal layout is locked. */
html,body{margin:0;padding:0;background:#000;color:#fff;font-family:'Rajdhani',sans-serif;
  overflow:hidden;height:100vh;width:100vw;
  display:flex;align-items:center;justify-content:center}

.dash-frame{
  width:1280px;height:720px;flex:none;position:relative;
  transform-origin:center center;          /* flex-centered; JS scales to fit (fitStage) */
  overflow:hidden;                         /* clip any minor internal overflow */
  display:flex;flex-direction:column;
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
.gauge-val.big{font-size:1.9em;bottom:16px}
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
.bottom-bar{display:grid;grid-template-columns:1fr 1fr 1fr 1fr 1fr;gap:18px;
  padding:10px 18px;background:linear-gradient(180deg,#111214,#0d0e10);
  border-top:1px solid #2a2520}
.bot-cell{text-align:center;padding:8px 6px;border-radius:6px;
  background:rgba(255,255,255,0.03);border:1px solid #2a2a2a;
  display:flex;flex-direction:column;align-items:center;gap:4px}
.bot-val{font-family:'Orbitron';font-size:2.1em;font-weight:900;line-height:1}
.bot-label{font-family:'Orbitron';font-size:0.6em;letter-spacing:3px;color:#888;margin-top:2px}
.bot-bar{width:92%;height:14px;background:#1a1a1a;border-radius:7px;overflow:hidden;
  border:1px solid #2a2a2a;margin-top:4px}
.bot-bar-fill{height:100%;width:0%;border-radius:7px;transition:width .15s linear}

/* Battery — one large gauge per track, in that track's panel footer */
.bat-gauge{width:150px;height:48px;border:3px solid #555;border-radius:4px;
  position:relative;display:inline-block;vertical-align:middle}
.bat-gauge::after{content:'';position:absolute;right:-7px;top:16px;width:6px;height:16px;
  background:#555;border-radius:0 2px 2px 0}
.bat-fill{position:absolute;inset:3px;border-radius:2px;transition:width 0.5s ease}
.bat-footer{grid-column:1/-1;text-align:center;margin-top:10px;
  border-top:1px solid #2a2520;padding-top:8px}
.bat-cap{font-family:'Orbitron';font-size:1.2em;font-weight:900;margin-top:6px}
.bat-cap .bv{color:#9aa;font-size:0.7em;font-weight:700}
.bat-foot-label{font-family:'Orbitron';font-size:0.5em;letter-spacing:3px;color:#664;margin-top:3px}

/* Mode pills */
.mode-bar{display:flex;justify-content:center;gap:12px;padding:6px 0}
.mode-pill{padding:7px 24px;font-family:'Orbitron';font-size:0.95em;font-weight:700;
  letter-spacing:2px;background:rgba(255,255,255,0.02);color:#444;
  border:2px solid #222;border-radius:5px;transition:all 0.3s}
.mode-pill.on-rc{background:rgba(0,200,255,0.12);color:#0cf;border-color:#0cf;box-shadow:0 0 10px rgba(0,200,255,0.15)}
.mode-pill.on-joy{background:rgba(255,140,0,0.12);color:#f80;border-color:#f80;box-shadow:0 0 10px rgba(255,140,0,0.15)}
.mode-pill.on-blend{background:rgba(180,100,255,0.12);color:#b8f;border-color:#b8f;box-shadow:0 0 10px rgba(180,100,255,0.15)}

/* Speed gauge in bottom */
.speed-big{font-family:'Orbitron';font-size:1.6em;font-weight:900;color:#fff;
  text-shadow:0 0 15px rgba(255,255,255,0.3)}

/* Gear badge */
.gear-badge{display:inline-block;padding:6px 18px;font-family:'Orbitron';font-size:1.2em;
  font-weight:900;letter-spacing:4px;border:2px solid;border-radius:4px;margin-top:6px}
.g-eco{color:#0c6;border-color:#0c6;background:rgba(0,200,100,0.08)}
.g-norm{color:#fc0;border-color:#fc0;background:rgba(255,200,0,0.08)}
.g-turbo{color:#f44;border-color:#f44;background:rgba(255,60,60,0.08);animation:tp .5s infinite}
@keyframes tp{50%{box-shadow:0 0 12px rgba(255,60,60,0.4)}}

/* Responsive */
/* Responsive media query removed — stage is fixed 1280x720, viewport-scaled. */
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
        <div class="gauge-val big" id="vRL" style="color:#0cf">1280</div>
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
        <div class="gauge-val" id="vCL" style="color:#0cf">0W</div>
      </div>
      <div class="gauge-label">POWER</div>
    </div>

    <div class="bat-footer">
      <div class="bat-gauge"><div class="bat-fill" id="batLFill" style="width:80%;background:linear-gradient(90deg,#0c6,#4f8)"></div></div>
      <div class="bat-cap"><span id="batLPct" style="color:#0c6">82%</span> <span class="bv" id="batLV">11.4V</span></div>
      <div class="bat-foot-label">LEFT BATTERY</div>
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

    <!-- Battery moved to a large footer under each track panel. -->
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
        <div class="gauge-val big" id="vRR" style="color:#f80">1315</div>
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
        <div class="gauge-val" id="vCR" style="color:#f80">0W</div>
      </div>
      <div class="gauge-label">POWER</div>
    </div>

    <div class="bat-footer">
      <div class="bat-gauge"><div class="bat-fill" id="batRFill" style="width:78%;background:linear-gradient(90deg,#0c6,#4f8)"></div></div>
      <div class="bat-cap"><span id="batRPct" style="color:#0c6">78%</span> <span class="bv" id="batRV">11.2V</span></div>
      <div class="bat-foot-label">RIGHT BATTERY</div>
    </div>
  </div>

</div>

<div class="bottom-bar">
  <div class="bot-cell">
    <div class="bot-val" id="thrL" style="color:#0cf">0%</div>
    <div class="bot-bar"><div class="bot-bar-fill" id="thrLBar" style="background:linear-gradient(90deg,#1f6f8b,#0cf)"></div></div>
    <div class="bot-label">THROTTLE L</div>
  </div>
  <div class="bot-cell">
    <div class="bot-val" id="curTot" style="color:#f80">0A</div>
    <div class="bot-bar"><div class="bot-bar-fill" id="curTotBar" style="background:linear-gradient(90deg,#f80,#fc0)"></div></div>
    <div class="bot-label">TOTAL AMPS</div>
  </div>
  <div class="bot-cell">
    <div class="bot-val" id="pwrTot" style="color:#fc0">0W</div>
    <div class="bot-bar"><div class="bot-bar-fill" id="pwrTotBar" style="background:linear-gradient(90deg,#fc0,#f44)"></div></div>
    <div class="bot-label">TOTAL POWER</div>
  </div>
  <div class="bot-cell">
    <div class="bot-val" id="curTotR" style="color:#aaa">0:00</div>
    <div class="bot-label" style="margin-top:8px">UPTIME</div>
  </div>
  <div class="bot-cell">
    <div class="bot-val" id="thrR" style="color:#c8f">0%</div>
    <div class="bot-bar"><div class="bot-bar-fill" id="thrRBar" style="background:linear-gradient(90deg,#7a3f8a,#c8f)"></div></div>
    <div class="bot-label">THROTTLE R</div>
  </div>
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
// ── Live telemetry client — robust SSE with watchdog + explicit recovery ──
const AP_IP = '192.168.4.1';
const SAME_ORIGIN = (location.protocol === 'http:' && location.host);
const EVENTS_URL = SAME_ORIGIN ? '/events' : 'http://' + AP_IP + '/events';
const STALE_MS = 1500;          // no frame → flip to OFFLINE
const WATCHDOG_MS = 3000;       // silent SSE → force reconnect (defeats Safari stall)
const RECONNECT_DELAY_MS = 1500;
const ESC_STALE_AGE_MS = 1500;  // per-ESC freshness threshold

const MR=6000, GR=20, WC=0.95, PWR_MAX=200;
const gn=['ECO','NORMAL','BOOST'], gcl=['g-eco','g-norm','g-turbo'];

let eL={r:0,c:0,b:11.4,tM:25,tE:25}, eR={r:0,c:0,b:11.2,tM:25,tE:25};
let outL=1500, outR=1500;
let lastRx=0, lastFrameAt=0, uptime0=0;
let es=null, connState='INIT', reconnectTimer=null;
let rafScheduled=false;

// Smoothed display state — eases toward the latest telemetry each animation
// frame so gauges/numbers glide (soft curve) instead of snapping between the
// 5 Hz samples. Discrete fields (gear/mode/direction) stay instant.
let dL={r:0,c:0,b:11.4,tM:25,tE:25}, dR={r:0,c:0,b:11.2,tM:25,tE:25};
let dOutL=1500, dOutR=1500;
const SMOOTH=0.18;                       // ease factor per frame (higher = snappier)
function ez(c,t){ return c + (t - c) * SMOOTH; }

function bp(v){return Math.max(0,Math.min(100,((v-9)/(12.6-9))*100))}
function bc(p){return p>50?'linear-gradient(90deg,#0c6,#4f8)':p>25?'linear-gradient(90deg,#fa0,#fc0)':'linear-gradient(90deg,#f44,#f66)'}
function ndl(id,val,max){const e=document.getElementById(id);
  if(e) e.setAttribute('transform',`rotate(${-135+Math.max(0,Math.min(val/max,1))*270},50,50)`);}
function tc(t){return t<55?'#0c6':t<80?'#fc0':'#f44'}
function set(id,txt){const e=document.getElementById(id); if(e) e.textContent=txt;}

// ── Connection state machine ──────────────────────────────────────────
const STATE_TEXT = {INIT:'INIT', CONNECTING:'CONNECTING…', LIVE:'LIVE',
  RECONNECTING:'RECONNECTING…', OFFLINE:'OFFLINE', ERROR:'ERROR'};
const STATE_COLOR = {CONNECTING:'#fc0', LIVE:'#0f0', RECONNECTING:'#fa0',
  OFFLINE:'#f44', ERROR:'#f44'};

function setConnState(state, msg){
  if(connState===state && !msg) return;
  connState = state;
  set('conn', STATE_TEXT[state]||state);
  const dot=document.querySelector('.dot-live');
  if(dot){const c=STATE_COLOR[state]||'#888';
    dot.style.background=c; dot.style.boxShadow='0 0 6px '+c;}
  if(msg) showBanner(msg, state==='LIVE'?'ok':'err'); else if(state==='LIVE') hideBanner();
}

function showBanner(msg, kind){
  let b=document.getElementById('errBanner');
  if(!b){
    b=document.createElement('div'); b.id='errBanner';
    b.style.cssText='position:fixed;top:0;left:0;right:0;color:#fff;padding:6px 10px;'+
      'font:11px sans-serif;text-align:center;z-index:9999;letter-spacing:1px';
    document.body.appendChild(b);
  }
  b.style.background = kind==='ok' ? '#1f6f8b' : '#c0392b';
  b.textContent = msg;
  b.style.display = 'block';
}
function hideBanner(){const b=document.getElementById('errBanner'); if(b) b.style.display='none';}

// ── Render — continuous requestAnimationFrame loop (eases displayed values ~60fps) ──
function render(){
  // Ease displayed values toward the latest targets (soft exponential curve)
  // so gauges + numbers glide between the 5 Hz telemetry samples.
  dL.r=ez(dL.r,eL.r); dL.c=ez(dL.c,eL.c); dL.b=ez(dL.b,eL.b); dL.tE=ez(dL.tE,eL.tE); dL.tM=ez(dL.tM,eL.tM);
  dR.r=ez(dR.r,eR.r); dR.c=ez(dR.c,eR.c); dR.b=ez(dR.b,eR.b); dR.tE=ez(dR.tE,eR.tE); dR.tM=ez(dR.tM,eR.tM);
  dOutL=ez(dOutL,outL); dOutR=ez(dOutR,outR);

  ndl('ndlEL',dL.tE,120);ndl('ndlML',dL.tM,120);ndl('ndlRL',dL.r,MR);ndl('ndlCL',dL.c*dL.b,PWR_MAX);
  ndl('ndlER',dR.tE,120);ndl('ndlMR',dR.tM,120);ndl('ndlRR',dR.r,MR);ndl('ndlCR',dR.c*dR.b,PWR_MAX);
  set('vEL',dL.tE.toFixed(0)+'°');set('vML',dL.tM.toFixed(0)+'°');
  set('vRL',Math.round(dL.r));set('vCL',Math.round(Math.max(0,dL.c*dL.b))+'W');
  set('vER',dR.tE.toFixed(0)+'°');set('vMR',dR.tM.toFixed(0)+'°');
  set('vRR',Math.round(dR.r));set('vCR',Math.round(Math.max(0,dR.c*dR.b))+'W');
  [['vEL',dL.tE],['vML',dL.tM],['vER',dR.tE],['vMR',dR.tM]].forEach(([id,t])=>{
    const e=document.getElementById(id); if(e) e.style.color=tc(t);});
  // Vehicle speed — average of SIGNED per-track velocities. A 360 pivot
  // (one track fwd, one rev) averages to ~0; 30%+100% fwd averages to ~65%.
  const sgnL = dOutL>1530?1:(dOutL<1470?-1:0), sgnR = dOutR>1530?1:(dOutR<1470?-1:0);
  const vehKmh = Math.abs((dL.r*sgnL + dR.r*sgnR)/2 / GR * WC * 60/1000);
  set('spd', vehKmh.toFixed(1));

  // Per-battery percentages — each gets its own number and indicator color
  const bL=bp(dL.b), bR=bp(dR.b);
  const batColor = p => p>50?'#0c6' : p>25?'#fc0' : '#f44';
  const fL=document.getElementById('batLFill'), fR=document.getElementById('batRFill');
  if(fL){fL.style.width=bL+'%'; fL.style.background=bc(bL);}
  if(fR){fR.style.width=bR+'%'; fR.style.background=bc(bR);}
  set('batLV',dL.b.toFixed(1)+'V'); set('batRV',dR.b.toFixed(1)+'V');
  set('batLPct',bL.toFixed(0)+'%'); set('batRPct',bR.toFixed(0)+'%');
  const lpEl=document.getElementById('batLPct'); if(lpEl) lpEl.style.color=batColor(bL);
  const rpEl=document.getElementById('batRPct'); if(rpEl) rpEl.style.color=batColor(bR);
  // Bottom row — values + bar gauges. Throttle is signed (-100..+100), bar
  // fills by magnitude. Amps/Watts scaled to typical driving range.
  const tL = Math.round((dOutL-1500)/5), tR = Math.round((dOutR-1500)/5);
  set('thrL', tL+'%'); set('thrR', tR+'%');
  const AMPS_MAX = 60, WATTS_MAX = 2*PWR_MAX;   // total = both motors; stays in sync with per-motor scale
  const amps = dL.c+dR.c, watts = dL.c*dL.b + dR.c*dR.b;
  set('curTot', amps.toFixed(1)+'A');
  set('pwrTot', Math.round(watts)+'W');
  const setBar=(id,pct)=>{const e=document.getElementById(id);
    if(e) e.style.width = Math.max(0,Math.min(100,pct))+'%';};
  setBar('thrLBar', Math.abs(tL));
  setBar('thrRBar', Math.abs(tR));
  setBar('curTotBar', amps/AMPS_MAX*100);
  setBar('pwrTotBar', watts/WATTS_MAX*100);
  requestAnimationFrame(render);          // continuous loop — keeps easing at ~60 fps
}
// applyData no longer needs to schedule a render; the continuous loop above
// always reflects the latest targets. Kept as a no-op for call-site safety.
function scheduleRender(){}

function dirArrow(o){return o>1530?1:(o<1470?-1:0);}

// Combined heading arrow: derive a 2D vehicle heading from the two track PWMs.
//   fwd  = (L+R)/2   forward(+)/reverse(-) component
//   turn = (R-L)/2   right track faster (+) = turning right (sign verified on-device)
// Rotate the up-arrow clockwise by atan2(turn,fwd): 0=forward(up), +90=right,
// -90=left, ±180=reverse(down); forward-right ≈ +45. Colour green→orange→red by
// how forward-vs-reverse the heading is. CSS transition animates it smoothly.
let vDirAngle = 0;
function lerp(a,b,t){return a+(b-a)*t;}
function dirColor(t){            // t: 0 forward(green) … 0.5 sideways(orange) … 1 reverse(red)
  t=Math.max(0,Math.min(1,t));
  const g=[0,200,100], o=[255,136,0], r=[255,68,68];
  const c = t<0.5 ? [lerp(g[0],o[0],t*2),lerp(g[1],o[1],t*2),lerp(g[2],o[2],t*2)]
                  : [lerp(o[0],r[0],(t-0.5)*2),lerp(o[1],r[1],(t-0.5)*2),lerp(o[2],r[2],(t-0.5)*2)];
  return 'rgb('+c.map(Math.round).join(',')+')';
}
function applyDir(){
  [['dirL',dirArrow(outL)],['dirR',dirArrow(outR)]].forEach(([id,d])=>{
    const e=document.getElementById(id); if(!e) return;
    e.className='dir-arrows '+(d>0?'fwd':d<0?'rev':'off');
    e.innerHTML=d<0?'&#9660;&#9660;':'&#9650;&#9650;';});
  const vd=document.getElementById('vDir'); if(!vd) return;
  const L=outL-1500, R=outR-1500;
  const fwd=(L+R)/2, turn=(R-L)/2, mag=Math.hypot(fwd,turn);  // +turn = turning right (sign verified on-device)
  if(mag<25){                                    // deadzone — stationary, dim, hold last heading
    vd.className='vehicle-dir off'; vd.style.color='#333'; vd.style.textShadow='none';
    return;
  }
  const theta=Math.atan2(turn,fwd)*180/Math.PI;  // 0=fwd(up), +90=right, ±180=reverse(down)
  let dlt=(theta-vDirAngle)%360; if(dlt>180)dlt-=360; if(dlt<-180)dlt+=360;
  vDirAngle+=dlt;                                 // unwrap so it rotates the short way
  const col=dirColor((1-fwd/mag)/2);
  vd.className='vehicle-dir';
  vd.style.transform='rotate('+vDirAngle.toFixed(1)+'deg)';
  vd.style.color=col;
  vd.style.textShadow='0 0 14px '+col;
  vd.innerHTML='&#9650;';
}

function applyData(d){
  const s=v=>v/10;
  // Use the firmware-provided age field (ms) for per-ESC freshness — falls
  // back to 0 if missing (older firmware).
  const e0Fresh = d.e0 && d.e0.ok && (d.e0.age||0) < ESC_STALE_AGE_MS;
  const e1Fresh = d.e1 && d.e1.ok && (d.e1.age||0) < ESC_STALE_AGE_MS;
  if(e0Fresh){eL.r=d.e0.rpm; eL.c=s(d.e0.cur); eL.b=s(d.e0.v); eL.tE=d.e0.tE; eL.tM=d.e0.tM;}
  if(e1Fresh){eR.r=d.e1.rpm; eR.c=s(d.e1.cur); eR.b=s(d.e1.v); eR.tE=d.e1.tE; eR.tM=d.e1.tM;}
  outL=d.outL; outR=d.outR;

  const gb=document.getElementById('gearBadge');
  if(gb){gb.textContent=gn[d.gear]||'--'; gb.className='gear-badge '+(gcl[d.gear]||'g-norm');}
  const mp=document.getElementById('mRC'); if(mp) mp.className='mode-pill'+(d.mode===0?' on-rc':'');
  const mj=document.getElementById('mJoy'); if(mj) mj.className='mode-pill'+(d.mode===1?' on-joy':'');
  const mb=document.getElementById('mBlend'); if(mb) mb.className='mode-pill'+(d.mode===2?' on-blend':'');

  applyDir();

  // Per-ESC freshness shown on the panels with last-update age (seconds)
  const pL=document.querySelector('.track-left'), pR=document.querySelector('.track-right');
  if(pL){pL.style.opacity = e0Fresh?'1':'0.35';
    pL.title = e0Fresh ? 'ESC0 live' : ('ESC0 stale '+((d.e0?.age||0)/1000).toFixed(1)+'s');}
  if(pR){pR.style.opacity = e1Fresh?'1':'0.35';
    pR.title = e1Fresh ? 'ESC1 live' : ('ESC1 stale '+((d.e1?.age||0)/1000).toFixed(1)+'s');}

  scheduleRender();
}

function handleFrame(text){
  let d;
  try{ d = JSON.parse(text); }
  catch(e){ showBanner('JSON parse error: '+e.message, 'err'); return; }
  try{
    lastRx = Date.now(); lastFrameAt = lastRx;
    if(!uptime0) uptime0 = lastRx;
    applyData(d);
    if(connState !== 'LIVE') setConnState('LIVE');
    const up = Math.floor((Date.now()-uptime0)/1000);
    set('curTotR', Math.floor(up/60)+':'+String(up%60).padStart(2,'0'));
  }catch(e){ showBanner('Render error: '+(e.message||e), 'err'); }
}

// ── SSE connection management ─────────────────────────────────────────
function cleanupES(){
  if(es){ try{es.close();}catch(_){} es=null; }
  if(reconnectTimer){ clearTimeout(reconnectTimer); reconnectTimer=null; }
}
function scheduleReconnect(){
  if(reconnectTimer) return;
  reconnectTimer = setTimeout(()=>{ reconnectTimer=null; connect(); }, RECONNECT_DELAY_MS);
}
function connect(){
  cleanupES();
  setConnState('CONNECTING');
  try{ es = new EventSource(EVENTS_URL); }
  catch(e){ setConnState('ERROR','EventSource failed: '+e.message); scheduleReconnect(); return; }
  es.onmessage = (ev)=>handleFrame(ev.data);
  es.onerror = ()=>{ setConnState('RECONNECTING','Connection lost. Reconnecting in '+(RECONNECT_DELAY_MS/1000)+'s…'); scheduleReconnect(); };
}

// ── Watchdogs ─────────────────────────────────────────────────────────
function watchdog(){
  if(es && lastFrameAt && (Date.now()-lastFrameAt > WATCHDOG_MS)){
    showBanner('No data for '+((Date.now()-lastFrameAt)/1000).toFixed(1)+'s — forcing reconnect', 'err');
    cleanupES(); connect();
  }
}
function staleWatch(){
  if(connState==='LIVE' && Date.now()-lastRx > STALE_MS){
    setConnState('OFFLINE','Stream went quiet — waiting for next frame…');
  }
}

// ── Manual reconnect button (bottom-right) ────────────────────────────
(function addReconnectBtn(){
  const btn = document.createElement('button');
  btn.textContent = '⟳ Reconnect';
  btn.style.cssText = 'position:fixed;bottom:8px;right:8px;padding:6px 12px;background:#1f6f8b;'+
    'color:#fff;border:1px solid #2c8db0;border-radius:5px;font:11px sans-serif;'+
    'letter-spacing:1px;cursor:pointer;z-index:9998';
  btn.onclick = ()=>{ hideBanner(); cleanupES(); connect(); };
  document.body.appendChild(btn);
})();

connect();
requestAnimationFrame(render);            // start the continuous smoothing/render loop
setInterval(staleWatch, 500);
setInterval(watchdog, 1000);

// ── Fixed-ratio stage fitter ─────────────────────────────────────────
// The .dash-frame is fixed at 1280x720. We scale it uniformly so the
// shorter viewport dimension fits, then center it. Landscape fills the
// screen; portrait shrinks to fit the width and leaves empty space top/bot.
function fitStage(){
  const stage = document.querySelector('.dash-frame');
  if(!stage) return;
  const BW=1280, BH=720;
  // Scale the flex-centered stage to the SMALLER ratio so the whole thing fits
  // (portrait or landscape). visualViewport is the most reliable size on iOS.
  const vw = (window.visualViewport && window.visualViewport.width)  || window.innerWidth  || document.documentElement.clientWidth;
  const vh = (window.visualViewport && window.visualViewport.height) || window.innerHeight || document.documentElement.clientHeight;
  const s = Math.min(vw/BW, vh/BH);
  stage.style.transform = 'scale(' + s + ')';
}
window.addEventListener('resize', fitStage);
window.addEventListener('orientationchange', ()=>setTimeout(fitStage, 100));
if (window.visualViewport) window.visualViewport.addEventListener('resize', fitStage);
// Run now + delayed retries to catch iOS settling the viewport late.
fitStage(); setTimeout(fitStage, 200); setTimeout(fitStage, 600);
</script>
</body>
</html>

)DIGGER";
