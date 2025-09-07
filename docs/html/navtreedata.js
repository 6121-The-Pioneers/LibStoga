/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "LibStoga", "index.html", [
    [ "🤖 LibStoga", "index.html#autotoc_md3", [
      [ "⚙️ High-Performance PROS / C++ Robotics Framework by <b>6121D</b>", "index.html#autotoc_md4", null ],
      [ "📋 Table of Contents", "index.html#autotoc_md6", [
        [ "<b>📑 Detailed Sections:</b>", "index.html#autotoc_md8", null ]
      ] ],
      [ "📚 Documentation", "index.html#autotoc_md10", [
        [ "📖 Online Documentation", "index.html#autotoc_md11", null ],
        [ "🏗️ Local Documentation Generation", "index.html#autotoc_md12", null ],
        [ "🎨 Documentation Features", "index.html#autotoc_md13", null ]
      ] ],
      [ "2. Feature Overview", "index.html#autotoc_md16", null ],
      [ "🚀 Quick Start", "index.html#autotoc_md18", [
        [ "📦 Project Integration", "index.html#autotoc_md19", null ],
        [ "⚡ Minimal Example (Odometry + PID + Chassis)", "index.html#autotoc_md20", null ]
      ] ],
      [ "📐 Core Concepts", "index.html#autotoc_md22", [
        [ "📏 Angle", "index.html#autotoc_md23", null ],
        [ "📍 Point / Pose / Path", "index.html#autotoc_md24", null ]
      ] ],
      [ "5. Sensors &amp; Tracking", "index.html#autotoc_md26", [
        [ "TrackingWheel", "index.html#autotoc_md27", null ],
        [ "Odometry Backends", "index.html#autotoc_md28", [
          [ "ThreeWheelOdom", "index.html#autotoc_md29", null ],
          [ "ImuOdom", "index.html#autotoc_md30", null ]
        ] ],
        [ "Monte Carlo Localization (MCLTracker)", "index.html#autotoc_md31", null ],
        [ "Choosing a Tracker", "index.html#autotoc_md32", null ]
      ] ],
      [ "6. Controllers", "index.html#autotoc_md34", [
        [ "PID", "index.html#autotoc_md35", null ],
        [ "SmartPID (Adaptive)", "index.html#autotoc_md36", null ],
        [ "AdaptivePIDController (NEW)", "index.html#autotoc_md37", null ],
        [ "Auto-Tuning System (NEW)", "index.html#autotoc_md38", null ],
        [ "Joystick Curves &amp; Driver Control (NEW)", "index.html#autotoc_md39", null ],
        [ "Optional SD Card Data Persistence (NEW)", "index.html#autotoc_md40", null ],
        [ "LemLib-Style Movement Parameters (NEW)", "index.html#autotoc_md41", null ],
        [ "System Health Monitoring (NEW)", "index.html#autotoc_md42", null ]
      ] ],
      [ "7. Motion Layer", "index.html#autotoc_md44", [
        [ "Chassis API", "index.html#autotoc_md45", [
          [ "ChassisConfig (NEW)", "index.html#autotoc_md46", null ],
          [ "Threshold Exit Logic", "index.html#autotoc_md47", null ]
        ] ],
        [ "Pure Pursuit", "index.html#autotoc_md48", null ]
      ] ],
      [ "8. Autonomous Routine Handling", "index.html#autotoc_md50", null ],
      [ "8.5 Debug &amp; Testing Features (NEW)", "index.html#autotoc_md52", [
        [ "Odometry Diagnostics", "index.html#autotoc_md53", null ],
        [ "Sensor Configuration Testing", "index.html#autotoc_md54", null ],
        [ "Debug Output Control", "index.html#autotoc_md55", null ]
      ] ],
      [ "9. Tuning Guides", "index.html#autotoc_md57", [
        [ "Drivetrain PID (Lateral)", "index.html#autotoc_md58", null ],
        [ "Turn PID (Angular)", "index.html#autotoc_md59", null ],
        [ "Pure Pursuit Lookahead", "index.html#autotoc_md60", null ],
        [ "SmartPID Warning", "index.html#autotoc_md61", null ],
        [ "MCL Tips", "index.html#autotoc_md62", null ]
      ] ],
      [ "10. Extending the Library", "index.html#autotoc_md64", null ],
      [ "🐛 Troubleshooting", "index.html#autotoc_md66", null ],
      [ "12. Roadmap (Planned)", "index.html#autotoc_md68", null ],
      [ "13.5 Comprehensive Changelog", "index.html#autotoc_md70", [
        [ "📋 <b>Version History &amp; Release Notes</b>", "index.html#autotoc_md71", null ]
      ] ],
      [ "<b>v3.0.0 - Phase 1: Enhanced Control Systems</b> 🚀", "index.html#autotoc_md73", [
        [ "🎮 <b>Major Features: Joystick Curves &amp; Driver Control</b>", "index.html#autotoc_md74", [
          [ "<b>Advanced Joystick Curve System</b>", "index.html#autotoc_md75", null ],
          [ "<b>Configurable Drive Methods</b>", "index.html#autotoc_md76", null ],
          [ "<b>Performance Characteristics</b>", "index.html#autotoc_md77", null ]
        ] ],
        [ "💾 <b>Major Features: SD Card Data Persistence</b>", "index.html#autotoc_md79", [
          [ "<b>Completely Optional Architecture</b>", "index.html#autotoc_md80", null ],
          [ "<b>Tuning Profile Management</b>", "index.html#autotoc_md81", null ],
          [ "<b>Adaptive PID History Logging</b>", "index.html#autotoc_md82", null ],
          [ "<b>Match Data Recording</b>", "index.html#autotoc_md83", null ],
          [ "<b>Storage Management</b>", "index.html#autotoc_md84", null ]
        ] ],
        [ "🎯 <b>Major Features: LemLib-Style Movement Parameters</b>", "index.html#autotoc_md86", [
          [ "<b>Enhanced Parameter Structures</b>", "index.html#autotoc_md87", null ],
          [ "<b>Advanced Movement Control</b>", "index.html#autotoc_md88", null ],
          [ "<b>Movement Monitoring Framework</b>", "index.html#autotoc_md89", null ]
        ] ],
        [ "🏥 <b>Major Features: System Health Monitoring</b>", "index.html#autotoc_md91", [
          [ "<b>Comprehensive Health Detection</b>", "index.html#autotoc_md92", null ],
          [ "<b>Real-time Health System</b>", "index.html#autotoc_md93", null ],
          [ "<b>Health Status Classification</b>", "index.html#autotoc_md94", null ],
          [ "<b>Motor Health Metrics</b>", "index.html#autotoc_md95", null ]
        ] ]
      ] ],
      [ "<b>v2.1.0 - Adaptive Control &amp; Auto-Tuning</b> 🧠", "index.html#autotoc_md97", [
        [ "🎛️ <b>Adaptive PID Controller</b>", "index.html#autotoc_md98", null ],
        [ "🎯 <b>Auto-Tuning System</b>", "index.html#autotoc_md99", null ],
        [ "🛡️ <b>Safety Features</b>", "index.html#autotoc_md100", null ]
      ] ],
      [ "<b>v2.0.0 - Core Architecture Overhaul</b> 🏗️", "index.html#autotoc_md102", [
        [ "🔧 <b>Major Architecture Changes</b>", "index.html#autotoc_md103", null ],
        [ "📊 <b>Enhanced Odometry System</b>", "index.html#autotoc_md104", null ],
        [ "🎮 <b>Controller Integration</b>", "index.html#autotoc_md105", null ]
      ] ],
      [ "<b>v1.5.0 - MCL &amp; Localization</b> 🧭", "index.html#autotoc_md107", [
        [ "🎯 <b>Monte Carlo Localization</b>", "index.html#autotoc_md108", null ],
        [ "📍 <b>Pose Estimation</b>", "index.html#autotoc_md109", null ]
      ] ],
      [ "<b>v1.0.0 - Initial Release</b> 🎉", "index.html#autotoc_md111", [
        [ "🏗️ <b>Core Architecture</b>", "index.html#autotoc_md112", null ],
        [ "🎛️ <b>PID Control System</b>", "index.html#autotoc_md113", null ],
        [ "📏 <b>Geometry System</b>", "index.html#autotoc_md114", null ]
      ] ],
      [ "<b>📈 Performance Metrics &amp; Benchmarks</b>", "index.html#autotoc_md116", [
        [ "<b>v3.0.0 Performance Characteristics</b>", "index.html#autotoc_md117", null ],
        [ "<b>Optimization Achievements</b>", "index.html#autotoc_md118", null ]
      ] ],
      [ "<b>🔧 API Changes &amp; Breaking Changes</b>", "index.html#autotoc_md120", [
        [ "<b>v3.0.0 Breaking Changes</b>", "index.html#autotoc_md121", null ],
        [ "<b>Migration Guide</b>", "index.html#autotoc_md122", null ]
      ] ],
      [ "<b>🐛 Known Issues &amp; Limitations</b>", "index.html#autotoc_md124", [
        [ "<b>Current Limitations</b>", "index.html#autotoc_md125", null ],
        [ "<b>Resolved Issues</b>", "index.html#autotoc_md126", null ]
      ] ],
      [ "<b>🗺️ Future Roadmap (Detailed)</b>", "index.html#autotoc_md128", [
        [ "<b>Phase 2: Advanced Motion Control (Q4 2025)</b>", "index.html#autotoc_md129", null ],
        [ "<b>Phase 3: Sensor Fusion &amp; AI (Q1 2026)</b>", "index.html#autotoc_md130", null ],
        [ "<b>Phase 4: Competition Features (Q2 2026)</b>", "index.html#autotoc_md131", null ],
        [ "<b>Phase 5: Developer Experience (Q3 2026)</b>", "index.html#autotoc_md132", null ]
      ] ],
      [ "<b>📚 Configuration Examples</b>", "index.html#autotoc_md134", [
        [ "<b>Complete Robot Configuration</b>", "index.html#autotoc_md135", null ],
        [ "<b>Performance Tuning Configuration</b>", "index.html#autotoc_md136", null ]
      ] ],
      [ "<b>🤝 Contributing &amp; Development</b>", "index.html#autotoc_md138", [
        [ "<b>Development Setup</b>", "index.html#autotoc_md139", null ],
        [ "<b>Code Standards</b>", "index.html#autotoc_md140", null ],
        [ "<b>Testing Requirements</b>", "index.html#autotoc_md141", null ]
      ] ],
      [ "<b>📞 Support &amp; Resources</b>", "index.html#autotoc_md143", [
        [ "<b>Documentation</b>", "index.html#autotoc_md144", null ],
        [ "<b>Community</b>", "index.html#autotoc_md145", null ]
      ] ],
      [ "<b>🏆 Acknowledgments</b>", "index.html#autotoc_md147", [
        [ "<b>Contributors</b>", "index.html#autotoc_md148", null ],
        [ "<b>Inspirations</b>", "index.html#autotoc_md149", null ],
        [ "<b>Special Thanks</b>", "index.html#autotoc_md150", null ]
      ] ],
      [ "📚 License / Attribution", "index.html#autotoc_md153", [
        [ "Final Notes", "index.html#autotoc_md155", null ]
      ] ]
    ] ],
    [ "Deprecated List", "deprecated.html", null ],
    [ "Topics", "topics.html", "topics" ],
    [ "Namespaces", "namespaces.html", [
      [ "Namespace List", "namespaces.html", "namespaces_dup" ],
      [ "Namespace Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", null ],
        [ "Functions", "namespacemembers_func.html", null ],
        [ "Variables", "namespacemembers_vars.html", null ],
        [ "Typedefs", "namespacemembers_type.html", null ],
        [ "Enumerations", "namespacemembers_enum.html", null ]
      ] ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", "functions_vars" ],
        [ "Enumerations", "functions_enum.html", null ],
        [ "Enumerator", "functions_eval.html", null ],
        [ "Related Symbols", "functions_rela.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"Timer_8hpp.html",
"classls_1_1DataLogger.html",
"classls_1_1SmartPID.html#a3361b1251a5c3bc6682f9c69224e74c1",
"functions_func_b.html",
"structls_1_1ChassisConfig.html",
"structls_1_1SwingToPointParams.html"
];

var SYNCONMSG = 'click to disable panel synchronization';
var SYNCOFFMSG = 'click to enable panel synchronization';
var LISTOFALLMEMBERS = 'List of all members';