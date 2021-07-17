<!DOCTYPE html><html><head>
      <title>README</title>
      <meta charset="utf-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      
      <link rel="stylesheet" href="file:////home/phodor/.vscode/extensions/shd101wyy.markdown-preview-enhanced-0.5.22/node_modules/@shd101wyy/mume/dependencies/katex/katex.min.css">
      
      
      
      
      
      
      
      
      
      <style>
      /**
 * prism.js Github theme based on GitHub's theme.
 * @author Sam Clarke
 */
code[class*="language-"],
pre[class*="language-"] {
  color: #333;
  background: none;
  font-family: Consolas, "Liberation Mono", Menlo, Courier, monospace;
  text-align: left;
  white-space: pre;
  word-spacing: normal;
  word-break: normal;
  word-wrap: normal;
  line-height: 1.4;

  -moz-tab-size: 8;
  -o-tab-size: 8;
  tab-size: 8;

  -webkit-hyphens: none;
  -moz-hyphens: none;
  -ms-hyphens: none;
  hyphens: none;
}

/* Code blocks */
pre[class*="language-"] {
  padding: .8em;
  overflow: auto;
  /* border: 1px solid #ddd; */
  border-radius: 3px;
  /* background: #fff; */
  background: #f5f5f5;
}

/* Inline code */
:not(pre) > code[class*="language-"] {
  padding: .1em;
  border-radius: .3em;
  white-space: normal;
  background: #f5f5f5;
}

.token.comment,
.token.blockquote {
  color: #969896;
}

.token.cdata {
  color: #183691;
}

.token.doctype,
.token.punctuation,
.token.variable,
.token.macro.property {
  color: #333;
}

.token.operator,
.token.important,
.token.keyword,
.token.rule,
.token.builtin {
  color: #a71d5d;
}

.token.string,
.token.url,
.token.regex,
.token.attr-value {
  color: #183691;
}

.token.property,
.token.number,
.token.boolean,
.token.entity,
.token.atrule,
.token.constant,
.token.symbol,
.token.command,
.token.code {
  color: #0086b3;
}

.token.tag,
.token.selector,
.token.prolog {
  color: #63a35c;
}

.token.function,
.token.namespace,
.token.pseudo-element,
.token.class,
.token.class-name,
.token.pseudo-class,
.token.id,
.token.url-reference .token.variable,
.token.attr-name {
  color: #795da3;
}

.token.entity {
  cursor: help;
}

.token.title,
.token.title .token.punctuation {
  font-weight: bold;
  color: #1d3e81;
}

.token.list {
  color: #ed6a43;
}

.token.inserted {
  background-color: #eaffea;
  color: #55a532;
}

.token.deleted {
  background-color: #ffecec;
  color: #bd2c00;
}

.token.bold {
  font-weight: bold;
}

.token.italic {
  font-style: italic;
}


/* JSON */
.language-json .token.property {
  color: #183691;
}

.language-markup .token.tag .token.punctuation {
  color: #333;
}

/* CSS */
code.language-css,
.language-css .token.function {
  color: #0086b3;
}

/* YAML */
.language-yaml .token.atrule {
  color: #63a35c;
}

code.language-yaml {
  color: #183691;
}

/* Ruby */
.language-ruby .token.function {
  color: #333;
}

/* Markdown */
.language-markdown .token.url {
  color: #795da3;
}

/* Makefile */
.language-makefile .token.symbol {
  color: #795da3;
}

.language-makefile .token.variable {
  color: #183691;
}

.language-makefile .token.builtin {
  color: #0086b3;
}

/* Bash */
.language-bash .token.keyword {
  color: #0086b3;
}

/* highlight */
pre[data-line] {
  position: relative;
  padding: 1em 0 1em 3em;
}
pre[data-line] .line-highlight-wrapper {
  position: absolute;
  top: 0;
  left: 0;
  background-color: transparent;
  display: block;
  width: 100%;
}

pre[data-line] .line-highlight {
  position: absolute;
  left: 0;
  right: 0;
  padding: inherit 0;
  margin-top: 1em;
  background: hsla(24, 20%, 50%,.08);
  background: linear-gradient(to right, hsla(24, 20%, 50%,.1) 70%, hsla(24, 20%, 50%,0));
  pointer-events: none;
  line-height: inherit;
  white-space: pre;
}

pre[data-line] .line-highlight:before, 
pre[data-line] .line-highlight[data-end]:after {
  content: attr(data-start);
  position: absolute;
  top: .4em;
  left: .6em;
  min-width: 1em;
  padding: 0 .5em;
  background-color: hsla(24, 20%, 50%,.4);
  color: hsl(24, 20%, 95%);
  font: bold 65%/1.5 sans-serif;
  text-align: center;
  vertical-align: .3em;
  border-radius: 999px;
  text-shadow: none;
  box-shadow: 0 1px white;
}

pre[data-line] .line-highlight[data-end]:after {
  content: attr(data-end);
  top: auto;
  bottom: .4em;
}html body{font-family:"Helvetica Neue",Helvetica,"Segoe UI",Arial,freesans,sans-serif;font-size:16px;line-height:1.6;color:#333;background-color:#fff;overflow:initial;box-sizing:border-box;word-wrap:break-word}html body>:first-child{margin-top:0}html body h1,html body h2,html body h3,html body h4,html body h5,html body h6{line-height:1.2;margin-top:1em;margin-bottom:16px;color:#000}html body h1{font-size:2.25em;font-weight:300;padding-bottom:.3em}html body h2{font-size:1.75em;font-weight:400;padding-bottom:.3em}html body h3{font-size:1.5em;font-weight:500}html body h4{font-size:1.25em;font-weight:600}html body h5{font-size:1.1em;font-weight:600}html body h6{font-size:1em;font-weight:600}html body h1,html body h2,html body h3,html body h4,html body h5{font-weight:600}html body h5{font-size:1em}html body h6{color:#5c5c5c}html body strong{color:#000}html body del{color:#5c5c5c}html body a:not([href]){color:inherit;text-decoration:none}html body a{color:#08c;text-decoration:none}html body a:hover{color:#00a3f5;text-decoration:none}html body img{max-width:100%}html body>p{margin-top:0;margin-bottom:16px;word-wrap:break-word}html body>ul,html body>ol{margin-bottom:16px}html body ul,html body ol{padding-left:2em}html body ul.no-list,html body ol.no-list{padding:0;list-style-type:none}html body ul ul,html body ul ol,html body ol ol,html body ol ul{margin-top:0;margin-bottom:0}html body li{margin-bottom:0}html body li.task-list-item{list-style:none}html body li>p{margin-top:0;margin-bottom:0}html body .task-list-item-checkbox{margin:0 .2em .25em -1.8em;vertical-align:middle}html body .task-list-item-checkbox:hover{cursor:pointer}html body blockquote{margin:16px 0;font-size:inherit;padding:0 15px;color:#5c5c5c;background-color:#f0f0f0;border-left:4px solid #d6d6d6}html body blockquote>:first-child{margin-top:0}html body blockquote>:last-child{margin-bottom:0}html body hr{height:4px;margin:32px 0;background-color:#d6d6d6;border:0 none}html body table{margin:10px 0 15px 0;border-collapse:collapse;border-spacing:0;display:block;width:100%;overflow:auto;word-break:normal;word-break:keep-all}html body table th{font-weight:bold;color:#000}html body table td,html body table th{border:1px solid #d6d6d6;padding:6px 13px}html body dl{padding:0}html body dl dt{padding:0;margin-top:16px;font-size:1em;font-style:italic;font-weight:bold}html body dl dd{padding:0 16px;margin-bottom:16px}html body code{font-family:Menlo,Monaco,Consolas,'Courier New',monospace;font-size:.85em !important;color:#000;background-color:#f0f0f0;border-radius:3px;padding:.2em 0}html body code::before,html body code::after{letter-spacing:-0.2em;content:"\00a0"}html body pre>code{padding:0;margin:0;font-size:.85em !important;word-break:normal;white-space:pre;background:transparent;border:0}html body .highlight{margin-bottom:16px}html body .highlight pre,html body pre{padding:1em;overflow:auto;font-size:.85em !important;line-height:1.45;border:#d6d6d6;border-radius:3px}html body .highlight pre{margin-bottom:0;word-break:normal}html body pre code,html body pre tt{display:inline;max-width:initial;padding:0;margin:0;overflow:initial;line-height:inherit;word-wrap:normal;background-color:transparent;border:0}html body pre code:before,html body pre tt:before,html body pre code:after,html body pre tt:after{content:normal}html body p,html body blockquote,html body ul,html body ol,html body dl,html body pre{margin-top:0;margin-bottom:16px}html body kbd{color:#000;border:1px solid #d6d6d6;border-bottom:2px solid #c7c7c7;padding:2px 4px;background-color:#f0f0f0;border-radius:3px}@media print{html body{background-color:#fff}html body h1,html body h2,html body h3,html body h4,html body h5,html body h6{color:#000;page-break-after:avoid}html body blockquote{color:#5c5c5c}html body pre{page-break-inside:avoid}html body table{display:table}html body img{display:block;max-width:100%;max-height:100%}html body pre,html body code{word-wrap:break-word;white-space:pre}}.markdown-preview{width:100%;height:100%;box-sizing:border-box}.markdown-preview .pagebreak,.markdown-preview .newpage{page-break-before:always}.markdown-preview pre.line-numbers{position:relative;padding-left:3.8em;counter-reset:linenumber}.markdown-preview pre.line-numbers>code{position:relative}.markdown-preview pre.line-numbers .line-numbers-rows{position:absolute;pointer-events:none;top:1em;font-size:100%;left:0;width:3em;letter-spacing:-1px;border-right:1px solid #999;-webkit-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none}.markdown-preview pre.line-numbers .line-numbers-rows>span{pointer-events:none;display:block;counter-increment:linenumber}.markdown-preview pre.line-numbers .line-numbers-rows>span:before{content:counter(linenumber);color:#999;display:block;padding-right:.8em;text-align:right}.markdown-preview .mathjax-exps .MathJax_Display{text-align:center !important}.markdown-preview:not([for="preview"]) .code-chunk .btn-group{display:none}.markdown-preview:not([for="preview"]) .code-chunk .status{display:none}.markdown-preview:not([for="preview"]) .code-chunk .output-div{margin-bottom:16px}.scrollbar-style::-webkit-scrollbar{width:8px}.scrollbar-style::-webkit-scrollbar-track{border-radius:10px;background-color:transparent}.scrollbar-style::-webkit-scrollbar-thumb{border-radius:5px;background-color:rgba(150,150,150,0.66);border:4px solid rgba(150,150,150,0.66);background-clip:content-box}html body[for="html-export"]:not([data-presentation-mode]){position:relative;width:100%;height:100%;top:0;left:0;margin:0;padding:0;overflow:auto}html body[for="html-export"]:not([data-presentation-mode]) .markdown-preview{position:relative;top:0}@media screen and (min-width:914px){html body[for="html-export"]:not([data-presentation-mode]) .markdown-preview{padding:2em calc(50% - 457px + 2em)}}@media screen and (max-width:914px){html body[for="html-export"]:not([data-presentation-mode]) .markdown-preview{padding:2em}}@media screen and (max-width:450px){html body[for="html-export"]:not([data-presentation-mode]) .markdown-preview{font-size:14px !important;padding:1em}}@media print{html body[for="html-export"]:not([data-presentation-mode]) #sidebar-toc-btn{display:none}}html body[for="html-export"]:not([data-presentation-mode]) #sidebar-toc-btn{position:fixed;bottom:8px;left:8px;font-size:28px;cursor:pointer;color:inherit;z-index:99;width:32px;text-align:center;opacity:.4}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] #sidebar-toc-btn{opacity:1}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc{position:fixed;top:0;left:0;width:300px;height:100%;padding:32px 0 48px 0;font-size:14px;box-shadow:0 0 4px rgba(150,150,150,0.33);box-sizing:border-box;overflow:auto;background-color:inherit}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc::-webkit-scrollbar{width:8px}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc::-webkit-scrollbar-track{border-radius:10px;background-color:transparent}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc::-webkit-scrollbar-thumb{border-radius:5px;background-color:rgba(150,150,150,0.66);border:4px solid rgba(150,150,150,0.66);background-clip:content-box}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc a{text-decoration:none}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc ul{padding:0 1.6em;margin-top:.8em}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc li{margin-bottom:.8em}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .md-sidebar-toc ul{list-style-type:none}html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .markdown-preview{left:300px;width:calc(100% -  300px);padding:2em calc(50% - 457px -  150px);margin:0;box-sizing:border-box}@media screen and (max-width:1274px){html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .markdown-preview{padding:2em}}@media screen and (max-width:450px){html body[for="html-export"]:not([data-presentation-mode])[html-show-sidebar-toc] .markdown-preview{width:100%}}html body[for="html-export"]:not([data-presentation-mode]):not([html-show-sidebar-toc]) .markdown-preview{left:50%;transform:translateX(-50%)}html body[for="html-export"]:not([data-presentation-mode]):not([html-show-sidebar-toc]) .md-sidebar-toc{display:none}
/* Please visit the URL below for more information: */
/*   https://shd101wyy.github.io/markdown-preview-enhanced/#/customize-css */

      </style>
    </head>
    <body for="html-export">
      <div class="mume markdown-preview  ">
      <h1 class="mume-header" id="autonomous-drone-perception-system">Autonomous Drone Perception System</h1>

<p><span class="katex"><span class="katex-mathml"><math xmlns="http://www.w3.org/1998/Math/MathML"><semantics><mrow><mover accent="true"><mi>x</mi><mo>^</mo></mover></mrow><annotation encoding="application/x-tex">\hat{x}</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height:0.69444em;vertical-align:0em;"></span><span class="mord accent"><span class="vlist-t"><span class="vlist-r"><span class="vlist" style="height:0.69444em;"><span style="top:-3em;"><span class="pstrut" style="height:3em;"></span><span class="mord mathnormal">x</span></span><span style="top:-3em;"><span class="pstrut" style="height:3em;"></span><span class="accent-body" style="left:-0.22222em;"><span class="mord">^</span></span></span></span></span></span></span></span></span></span></p>
<h1 class="mume-header" id="overview">Overview</h1>

<p>This project develops a state estimation system for an autonomous drone using an Extended Kalman Filter (EKF), provides a C++ implementation of this system and performs adjustments of the system parameters in a flight simulator.</p>
<p>The system developed in this project is based upon a methodology by TELLEX, BROWN and LUPASHIN [1].</p>
<p>This project is part of Udacity&apos;s Autonomous Flight Engineer Nanodegree [2]. This README serves as a final report for the project.</p>
<br>
<img src="videos/step_03_a_scenario_08.gif" height="400">
<img src="images/ekf_pseudo_code.jpg" height="400">
<p><br><br></p>
<h1 class="mume-header" id="safety-first">Safety First!</h1>

<p>1 - This project is educational only. The methodology used in this project was only used in the Udacity C++ flight simulator and not validated on a real drone.</p>
<p>2 - Only tune the parameters of your drone by following the procedure prescribed by your drone&#x2019;s manufacturer.</p>
<p>3 - Make sure you comply with your local regulations before flying a drone.</p>
<p>4 - This project makes several assumptions which may not apply on a real drone.</p>
<h1 class="mume-header" id="install-run">Install &amp; Run</h1>

<p>1 - Clone this repository</p>
<pre class="language-text">$ mkdir -p /drone/projects/perception
$ cd /drone/projects/perception
$ git clone https://github.com/martin0004/drone_perception_system.git
</pre>
<p>2 - Install QTCreator and the GLUT libs</p>
<pre class="language-text">$ sudo apt install qtcreator
$ sudo apt install qtbase5-examples    # optional - qt examples
$ sudo apt install qtbase5-doc-html    # optional - qt examples documentation
$ sudo apt install freeglut3-dev
</pre>
<p>3 - Compile the project.</p>
<pre class="language-text">$ cd /drone/projects/perception    # Make sure you open qt from here when compiling
$ qtcreator                        # This will launch QTCreator

File &gt; Open File Or Project &gt; simulator/project/CPPSim.pro
Click on tab Edit on the left side panel of QTCreator
Right click on CPPSim &gt; Run qmake
Right click on CPPSim &gt; Run
</pre>
<p>4 - You should now see a drone hover. Right click on the simulation to switch between scenarios.</p>
<p>5 - Look into file <code>QuadEstimatorEKF.txt</code> to find all perception system parameters. If you change a value in this file and save it, the drone behavior will change in the simulation (no need to close the simulator, just update and save the file). Try to find better system parameters!</p>
<h1 class="mume-header" id="optional-udacity-starter-code">[Optional] Udacity Starter Code</h1>

<p>Udacity provided students with some starter code. This starter code can be installed with the following procedure.</p>
<pre class="language-text">mkdir -p /drone/projects/perception/udacity_starter_code
cd /drone/projects/perception/udacity_starter_code
git clone https://github.com/udacity/FCND-Estimation-CPP
</pre>
<h1 class="mume-header" id="udacity-c-flight-simulator">Udacity C++ Flight Simulator</h1>

<p>Parameters of the system developed in this project were tweaked by flying a drone in a series of scenarios in the Udacity C++ flight simulator [3]. The tuning scenarios are described in section &#x201C;Validation&#x201D;.</p>
<p>The flight simulator itself is a small QT application. The flying area is about 5 m x 5 m and the scenarios last only a few seconds. A contextual menu allows to switch between scenarios and  display charts of the drone state variables.</p>
<img src="videos/simulator_-_overview.gif" width="700">
<p><br><br></p>
<table>
<thead>
<tr>
<th>Simulator Command</th>
<th>Action</th>
</tr>
</thead>
<tbody>
<tr>
<td>MOUSE LEFT</td>
<td>Rotate</td>
</tr>
<tr>
<td>MOUSE LEFT + X</td>
<td>Pan</td>
</tr>
<tr>
<td>MOUSE LEFT + Z</td>
<td>Zoom</td>
</tr>
<tr>
<td>MOUSE RIGHT</td>
<td>Open contextual menu</td>
</tr>
<tr>
<td>UP</td>
<td>Apply force up.</td>
</tr>
<tr>
<td>DOWN</td>
<td>Apply force down.</td>
</tr>
<tr>
<td>LEFT</td>
<td>Apply force left.</td>
</tr>
<tr>
<td>RIGHT</td>
<td>Apply force right.</td>
</tr>
<tr>
<td>W</td>
<td>Apply force forward.</td>
</tr>
<tr>
<td>S</td>
<td>Apply force back.</td>
</tr>
<tr>
<td>C</td>
<td>Clear graphs.</td>
</tr>
<tr>
<td>R</td>
<td>Reset simulation.</td>
</tr>
<tr>
<td>SPACE</td>
<td>Pause simulation.</td>
</tr>
</tbody>
</table>
<h1 class="mume-header" id="symbols">Symbols</h1>

<table>
<thead>
<tr>
<th>Acronyms</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>CF</td>
<td>Complementary filter.</td>
</tr>
<tr>
<td>EKF</td>
<td>Extended Kalman filter.</td>
</tr>
<tr>
<td>GPS</td>
<td>Global positioning system.</td>
</tr>
<tr>
<td>IMU</td>
<td>Inertial measurement unit.</td>
</tr>
<tr>
<td>wrt</td>
<td>Abbreviation for &#x201C;with respect to&#x201D;.</td>
</tr>
</tbody>
</table>
<table>
<thead>
<tr>
<th>Physical Constants</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>g</td>
<td>Gravitational acceleration.</td>
</tr>
</tbody>
</table>
<table>
<thead>
<tr>
<th>Indices</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>x (no index)</td>
<td>True value of x (ground truth).</td>
</tr>
<tr>
<td>x_acc</td>
<td>Variable from IMU accelerometer measurement.</td>
</tr>
<tr>
<td>x_GPS</td>
<td>Variable from GPS measurement.</td>
</tr>
<tr>
<td>x_gyro</td>
<td>Variable from IMU gyroscope measurement.</td>
</tr>
<tr>
<td>x^b</td>
<td>Variable expressed in drone body frame.</td>
</tr>
<tr>
<td>x_tilde</td>
<td>Measured variable.</td>
</tr>
<tr>
<td>x_hat</td>
<td>Estimated variable.</td>
</tr>
<tr>
<td>x_bar</td>
<td>Predicted variable (intermediate EKF value).</td>
</tr>
<tr>
<td>x_t</td>
<td>Variable at time step t.</td>
</tr>
<tr>
<td>x_{t-1}</td>
<td>Variable at time step t-1.</td>
</tr>
</tbody>
</table>
<table>
<thead>
<tr>
<th>Measurement</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>x_tilde</td>
<td>GPS measurement - x location - global frame.</td>
</tr>
<tr>
<td>y_tilde</td>
<td>GPS measurement - y location - global frame.</td>
</tr>
<tr>
<td>z_tilde</td>
<td>GPS measurement - z location - global frame.</td>
</tr>
<tr>
<td>x_dot_tilde, vx_tilde</td>
<td>GPS measurement - x speed - global frame.</td>
</tr>
<tr>
<td>y_dot_tilde, vy_tilde</td>
<td>GPS measurement - y speed - global frame.</td>
</tr>
<tr>
<td>z_dot_tilde, vz_tilde</td>
<td>GPS measurement - z speed - global frame.</td>
</tr>
<tr>
<td>x_ddot_tilde^b</td>
<td>IMU measurement - x acceleration - body frame.</td>
</tr>
<tr>
<td>y_ddot_tilde^b</td>
<td>IMU measurement - y acceleration - body frame.</td>
</tr>
<tr>
<td>z_ddot_tilde^b</td>
<td>IMU measurement - z acceleration - body frame.</td>
</tr>
<tr>
<td>p_tilde</td>
<td>IMU measurement - x body rate - body frame.</td>
</tr>
<tr>
<td>q_tilde</td>
<td>IMU measurement - y body rate - body frame.</td>
</tr>
<tr>
<td>r_tilde</td>
<td>IMU measurement - z body rate - body frame.</td>
</tr>
<tr>
<td>&#x3C8;_tilde</td>
<td>Magnetometer measurement - yaw - global frame.</td>
</tr>
</tbody>
</table>
<table>
<thead>
<tr>
<th>Errors</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>e_hat_x</td>
<td>True error of variable or vector x_hat.</td>
</tr>
<tr>
<td>e_hat_d</td>
<td>Position magnitude error.</td>
</tr>
<tr>
<td>e_hat_v</td>
<td>Velocity magnitude error.</td>
</tr>
<tr>
<td>e_hat_Euler</td>
<td>Max Euler error.</td>
</tr>
</tbody>
</table>
<table>
<thead>
<tr>
<th>State Variables</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>x (*)</td>
<td>State vector.</td>
</tr>
<tr>
<td>x (*)</td>
<td>Drone x position - global frame.</td>
</tr>
<tr>
<td>y</td>
<td>Drone y position - global frame.</td>
</tr>
<tr>
<td>z</td>
<td>Drone z position - global frame.</td>
</tr>
<tr>
<td>x_dot, vx (**)</td>
<td>Drone x speed - global frame.</td>
</tr>
<tr>
<td>y_dot, vy (**)</td>
<td>Drone y speed - global frame.</td>
</tr>
<tr>
<td>z_dot, vz (**)</td>
<td>Drone z speed - global frame.</td>
</tr>
<tr>
<td>&#x424;</td>
<td>Drone attitude about x - global frame.</td>
</tr>
<tr>
<td>&#x3B8;</td>
<td>Drone attitude about y - global frame.</td>
</tr>
<tr>
<td>&#x3C8;</td>
<td>Drone attitude about z - global frame.</td>
</tr>
</tbody>
</table>
<p>(*) Depending on the context, it will be obvious if x represents a vector or a position.<br>
(**) Notation <span class="katex"><span class="katex-mathml"><math xmlns="http://www.w3.org/1998/Math/MathML"><semantics><mrow><mi>v</mi><mi>x</mi></mrow><annotation encoding="application/x-tex">vx</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height:0.43056em;vertical-align:0em;"></span><span class="mord mathnormal">vx</span></span></span></span> will be used when <span class="katex"><span class="katex-mathml"><math xmlns="http://www.w3.org/1998/Math/MathML"><semantics><mrow><mover accent="true"><mi>x</mi><mo>&#x2D9;</mo></mover></mrow><annotation encoding="application/x-tex">\dot{x}</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height:0.66786em;vertical-align:0em;"></span><span class="mord accent"><span class="vlist-t"><span class="vlist-r"><span class="vlist" style="height:0.66786em;"><span style="top:-3em;"><span class="pstrut" style="height:3em;"></span><span class="mord mathnormal">x</span></span><span style="top:-3em;"><span class="pstrut" style="height:3em;"></span><span class="accent-body" style="left:-0.11111000000000001em;"><span class="mord">&#x2D9;</span></span></span></span></span></span></span></span></span></span> is difficult to read (e.g. when <span class="katex"><span class="katex-mathml"><math xmlns="http://www.w3.org/1998/Math/MathML"><semantics><mrow><mi>v</mi><mi>x</mi></mrow><annotation encoding="application/x-tex">vx</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height:0.43056em;vertical-align:0em;"></span><span class="mord mathnormal">vx</span></span></span></span> is used as an index).</p>
<table>
<thead>
<tr>
<th>Filters</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>g</td>
<td>EKF process model (called &quot;transition model&quot; in lectures)</td>
</tr>
<tr>
<td>g&apos;</td>
<td>Function which derives the Jacobian of g.</td>
</tr>
<tr>
<td>G</td>
<td>Jacobian of g.</td>
</tr>
<tr>
<td>h</td>
<td>EKF measurement model.</td>
</tr>
<tr>
<td>h&apos;</td>
<td>Function which derives the Jacobian of h.</td>
</tr>
<tr>
<td>H</td>
<td>Jacobian of h.</td>
</tr>
<tr>
<td>Q</td>
<td>Process noise covariance matrix.</td>
</tr>
<tr>
<td>R_bg</td>
<td>Rotation matrix from body frame to world frame.</td>
</tr>
<tr>
<td>R_bg[0:]</td>
<td>First line of R_bg.</td>
</tr>
<tr>
<td>R_bg&apos;</td>
<td>Derivative of R_bg wrt to yaw.</td>
</tr>
<tr>
<td>R_bg&apos;[0:]u[0:3]</td>
<td>Dot product of first line of R_bg by first 3 elements of u.</td>
</tr>
<tr>
<td>R_GPS</td>
<td>GPS measurement covariance matrix.</td>
</tr>
<tr>
<td>R_mag</td>
<td>Magnetometer measurement covariance matrix.</td>
</tr>
<tr>
<td>u</td>
<td>Command vector.</td>
</tr>
<tr>
<td>u[0:3]</td>
<td>First 3 elements of u.</td>
</tr>
<tr>
<td>x</td>
<td>State vector.</td>
</tr>
<tr>
<td>z</td>
<td>Measurement vector.</td>
</tr>
<tr>
<td>T_s</td>
<td>Complementary filter sampling period.</td>
</tr>
<tr>
<td>w_i</td>
<td>Complementary filter weight for sensor measurement i.</td>
</tr>
<tr>
<td>&#x3A3;_bar</td>
<td>EKF predicted state covariance matrix.</td>
</tr>
<tr>
<td>&#x3A3;_hat</td>
<td>EKF estimated state covariance matrix.</td>
</tr>
<tr>
<td>&#x3BD;</td>
<td>Noise probability distribution.</td>
</tr>
<tr>
<td>&#x3C3;_x</td>
<td>Standard deviation of variable x.</td>
</tr>
<tr>
<td>&#x3C4;</td>
<td>Complementary filter time constant.</td>
</tr>
</tbody>
</table>
<h1 class="mume-header" id="units">Units</h1>

<table>
<thead>
<tr>
<th>Units</th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td>m</td>
<td>Distance.</td>
</tr>
<tr>
<td>kg</td>
<td>Mass.</td>
</tr>
<tr>
<td>s</td>
<td>Zoom.</td>
</tr>
<tr>
<td>Hz</td>
<td>Rate (frequency).</td>
</tr>
<tr>
<td>rad</td>
<td>Angle.</td>
</tr>
</tbody>
</table>
<h1 class="mume-header" id="coordinate-frames">Coordinate Frames</h1>

<h5 class="mume-header" id="body-frame-world-frame-propeller-convention">Body Frame, World Frame, Propeller Convention</h5>

<p>This project uses the same <strong>body frame</strong>, <strong>world frame</strong> and <strong>propeller sign convention</strong> as in the Drone Control System project of this nanodegree. See reference [4] for a descriptions of these frames.</p>
<h5 class="mume-header" id="rotation-matrix-r_bg">Rotation Matrix R_bg</h5>

<p>This project use rotation matrix Rbg, which is the rotation matrix from the body frame to the world frame [1]. R_bg is a function of Euler angles &#x424; (pitch), &#x3B8; (roll), &#x3C8; (yaw).</p>
<p>Euler angles &#x424;, &#x3B8;, &#x3C8; are provided as input to any estimator which needs to derive R internally.</p>
<br>
<img src="images/rotation_matrix_-_fully_developped.jpg" width="700">
<br>
<h1 class="mume-header" id="vehicule">Vehicule</h1>

<h5 class="mume-header" id="architecture">Architecture</h5>

<p>The autonomous drone in this project uses the classical perception, planning and control architecture.</p>
<p>This project focuses on developing the <strong>perception</strong> system. The <strong>actuators</strong>, <strong>sensors</strong> and <strong>process</strong> (i.e. the drone itself) are already implemented in the Udacity C++ simulator used in this project [3]. The <strong>planning</strong> and <strong>control</strong> systems are also part of the simulator. A series of predefined trajectories will feed waypoints to the control system during each simulation scenario. However in the last step of this project, the control system is be replaced by the control system which was developed in the 3rd project of this Nanodegree [4].</p>
<br>
<img src="images/autonomy_architecture.jpg" width="700">
<br>
<br>
<h5 class="mume-header" id="sensors">Sensors</h5>

<table>
<thead>
<tr>
<th>Sensor</th>
<th>Measurement</th>
<th>Symbol</th>
<th>Rate (*)</th>
<th>Time Step (**)</th>
</tr>
</thead>
<tbody>
<tr>
<td>GPS</td>
<td>x location - global frame</td>
<td>x_tilde</td>
<td>10 Hz</td>
<td>0.1 s</td>
</tr>
<tr>
<td></td>
<td>y location - global frame</td>
<td>y_tilde</td>
<td>10 Hz</td>
<td>0.1 s</td>
</tr>
<tr>
<td></td>
<td>z location - global frame</td>
<td>z_tilde</td>
<td>10 Hz</td>
<td>0.1 s</td>
</tr>
<tr>
<td></td>
<td>x speed - global frame</td>
<td>x_dot_tilde, vx_tilde</td>
<td>10 Hz</td>
<td>0.1 s</td>
</tr>
<tr>
<td></td>
<td>y speed - global frame</td>
<td>y_dot_tilde, vy_tilde</td>
<td>10 Hz</td>
<td>0.1 s</td>
</tr>
<tr>
<td></td>
<td>z speed - global frame</td>
<td>z_dot_tilde, vz_tilde</td>
<td>10 Hz</td>
<td>0.1 s</td>
</tr>
<tr>
<td>IMU</td>
<td>x acceleration - body frame</td>
<td>x_ddot_tilde^b</td>
<td>500 Hz</td>
<td>0.002 s</td>
</tr>
<tr>
<td></td>
<td>y acceleration - body frame</td>
<td>y_ddot_tilde^b</td>
<td>500 Hz</td>
<td>0.002 s</td>
</tr>
<tr>
<td></td>
<td>z acceleration - body frame</td>
<td>z_ddot_tilde^b</td>
<td>500 Hz</td>
<td>0.002 s</td>
</tr>
<tr>
<td></td>
<td>x body rate - body frame</td>
<td>p_tilde</td>
<td>500 Hz</td>
<td>0.002 s</td>
</tr>
<tr>
<td></td>
<td>y body rate - body frame</td>
<td>q_tilde</td>
<td>500 Hz</td>
<td>0.002 s</td>
</tr>
<tr>
<td></td>
<td>z body rate - body frame</td>
<td>r_tilde</td>
<td>500 Hz</td>
<td>0.002 s</td>
</tr>
<tr>
<td>Magnetometer</td>
<td>Yaw</td>
<td>&#x3C8;_tilde</td>
<td>100 Hz</td>
<td>0.01 s</td>
</tr>
</tbody>
</table>
<p>(*) Rate = 1 / Time Step. <br><br>
(**) Cannot be faster than the controller time step (0.002 s).</p>
<p><br><br></p>
<h1 class="mume-header" id="perception-system">Perception System</h1>

<h3 class="mume-header" id="perception-system-overview">Perception System - Overview</h3>

<p>The perception system developed in this project is made of 2 estimators, which are described below.</p>
<p>Note that the inputs / outputs I used are based upon Udacity&#x2019;s C++ Flight Simulator internal workings [3]. These are a little different from the methodology presented in the lectures [2] and from article [1].</p>
<br>
<img src="images/perception_system_architecture.jpg" width="700">
<br>
<br>
<h3 class="mume-header" id="perception-system-estimated-state-vector">Perception System - Estimated State Vector</h3>

<p>The estimation system uses the following &#x201C;full&#x201D; vector system internally. Different estimators inside the system will update / use different state variables from this vector.</p>
<p>Note that the Euler angles order in this state vector is yaw (&#x3C8;), pitch (&#x3B8;), roll (&#x3A6;), which is the reverse of the conventional aerospace order. Note also that the yaw (&#x3C8;) update is shared between the attitude and position/speed estimators.</p>
<p>This state split is the methodology which was suggested in the Estimation course of the Nanodegree. It is a trade-off between implementation complexity, explainability and functionality. [5]</p>
<br>
<img src="images/state_vector.jpg" width="500">
<br><br>
<h1 class="mume-header" id="possible-improvements">Possible improvements</h1>

<ul>
<li>
<p>Using the report structure of this project as a template for improving reports of project #2 (drone planning system [8]) and project #3 (drone control system [PROJECT [3]).</p>
<ul>
<li>Adding an &#x201C;Autonomy Architecture&#x201D; diagram in project #2 and project #3</li>
<li>Adding a &#x201C;Planning System Architecture&#x201D; in project #2</li>
<li>Info from section &#x201C;C++ Implementation&#x201D; in this report which is common to project #4 and project #3 could be moved to project #3 to avoid duplication.</li>
</ul>
</li>
<li>
<p>In the &#x201C;Attitude Estimator&#x201D; and &#x201C;Position/Speed Estimator&#x201D; introduction, there is a small block diagram showing the inputs/outputs of each estimator. It would be great to modify these block diagrams so they show what happens in the inside of the estimator. As an example, the attitude estimator actually contains several internal steps: calculating the derivatives using a rotation matrix, sending roll and pitch to a complementary filter and integrating yaw only with dead reackoning. Each of these operations could be an internal block by itself with inputs /outputs. However these &#x201C;internal&#x201D; diagram are assumed beyond scope for now...</p>
</li>
<li>
<p>In section &#x201C;C++ Implementation&#x201D;</p>
<ul>
<li>Below chart &#x201C;Perception System&#x201D;, add small flowchart showing which method is being called by which method and at which frequency.</li>
</ul>
</li>
</ul>
<h1 class="mume-header" id="references">References</h1>

<p>[1] TELLEX, S., BROWN, A and LUPASHIN, S, Estimation for Quadrotors, 2021,<br>
<a href="https://www.overleaf.com/read/vymfngphcccj">https://www.overleaf.com/read/vymfngphcccj</a></p>
<p>[2] Udacity&apos;s Autonomous Flight Engineer Nanodegree. <a href="https://www.udacity.com/course/flying-car-nanodegree--nd787">https://www.udacity.com/course/flying-car-nanodegree--nd787</a></p>
<p>[3] Udacity, C++ Flight Simulator, github, <a href="https://github.com/udacity/FCND-Controls-CPP">https://github.com/udacity/FCND-Controls-CPP</a></p>
<p>[4] Martin Cote, Autonomous Drone Control System - Autonomous Flight Engineer Nanodegree - Project #3, <a href="https://github.com/martin0004/drone_control_system">https://github.com/martin0004/drone_control_system</a></p>
<p>[5] Udacity&apos;s Autonomous Flight Engineer Nanodegree, Course 4 - Estimation, Lesson 4 - The 3D EKF and UKF, Section 4 - EKF Tradeoffs 1 - State.</p>
<p>[6] Udacity&apos;s Autonomous Flight Engineer Nanodegree, Course 4 - Estimation, Lesson 4 - The 3D EKF and UKF, Section 6 - Attitude Estimation.</p>
<p>[7] Udacity&apos;s Autonomous Flight Engineer Nanodegree, Course 4 - Estimation, Lesson 2 - Introduction to Sensors, Section 10 - Full 3D Attitude Update.</p>
<p>[8] Martin Cote, Autonomous Drone Path Planning - Autonomous Flight Engineer Nanodegree - Project #2, <a href="https://github.com/martin0004/drone_path_planning">https://github.com/martin0004/drone_path_planning</a></p>

      </div>
      
      
    
    
    
    
    
    
    
    
  
    </body></html>