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
    [ "🤖 LibStoga", "index.html#autotoc_md0", [
      [ "⚙️ High-Performance PROS / C++ Robotics Framework by <b>6121D</b>", "index.html#autotoc_md1", null ],
      [ "📋 Table of Contents", "index.html#autotoc_md3", [
        [ "<b>📑 Detailed Sections:</b>", "index.html#autotoc_md5", null ]
      ] ],
      [ "🧠 Philosophy &amp; Goals", "index.html#autotoc_md7", null ],
      [ "2. Feature Overview", "index.html#autotoc_md9", null ],
      [ "🚀 Quick Start", "index.html#autotoc_md11", [
        [ "📦 Project Integration", "index.html#autotoc_md12", null ],
        [ "⚡ Minimal Example (Odometry + PID + Chassis)", "index.html#autotoc_md13", null ]
      ] ],
      [ "📐 Core Concepts", "index.html#autotoc_md15", [
        [ "📏 Angle", "index.html#autotoc_md16", null ],
        [ "📍 Point / Pose / Path", "index.html#autotoc_md17", null ]
      ] ],
      [ "5. Sensors &amp; Tracking", "index.html#autotoc_md19", [
        [ "TrackingWheel", "index.html#autotoc_md20", null ],
        [ "Odometry Backends", "index.html#autotoc_md21", [
          [ "ThreeWheelOdom", "index.html#autotoc_md22", null ],
          [ "ImuOdom", "index.html#autotoc_md23", null ]
        ] ],
        [ "Monte Carlo Localization (MCLTracker)", "index.html#autotoc_md24", null ],
        [ "Choosing a Tracker", "index.html#autotoc_md25", null ]
      ] ],
      [ "6. Controllers", "index.html#autotoc_md27", [
        [ "PID", "index.html#autotoc_md28", null ],
        [ "SmartPID (Adaptive)", "index.html#autotoc_md29", null ],
        [ "AdaptivePIDController (NEW)", "index.html#autotoc_md30", null ],
        [ "Auto-Tuning System (NEW)", "index.html#autotoc_md31", null ],
        [ "Joystick Curves &amp; Driver Control (NEW)", "index.html#autotoc_md32", null ],
        [ "Optional SD Card Data Persistence (NEW)", "index.html#autotoc_md33", null ],
        [ "LemLib-Style Movement Parameters (NEW)", "index.html#autotoc_md34", null ],
        [ "System Health Monitoring (NEW)", "index.html#autotoc_md35", null ]
      ] ],
      [ "7. Motion Layer", "index.html#autotoc_md37", [
        [ "Chassis API", "index.html#autotoc_md38", [
          [ "ChassisConfig (NEW)", "index.html#autotoc_md39", null ],
          [ "Threshold Exit Logic", "index.html#autotoc_md40", null ]
        ] ],
        [ "Pure Pursuit", "index.html#autotoc_md41", null ]
      ] ],
      [ "8. Autonomous Routine Handling", "index.html#autotoc_md43", null ],
      [ "8.5 Debug &amp; Testing Features (NEW)", "index.html#autotoc_md45", [
        [ "Odometry Diagnostics", "index.html#autotoc_md46", null ],
        [ "Sensor Configuration Testing", "index.html#autotoc_md47", null ],
        [ "Debug Output Control", "index.html#autotoc_md48", null ]
      ] ],
      [ "9. Tuning Guides", "index.html#autotoc_md50", [
        [ "Drivetrain PID (Lateral)", "index.html#autotoc_md51", null ],
        [ "Turn PID (Angular)", "index.html#autotoc_md52", null ],
        [ "Pure Pursuit Lookahead", "index.html#autotoc_md53", null ],
        [ "SmartPID Warning", "index.html#autotoc_md54", null ],
        [ "MCL Tips", "index.html#autotoc_md55", null ]
      ] ],
      [ "10. Extending the Library", "index.html#autotoc_md57", null ],
      [ "🐛 Troubleshooting", "index.html#autotoc_md59", null ],
      [ "12. Roadmap (Planned)", "index.html#autotoc_md61", null ],
      [ "13.5 Comprehensive Changelog", "index.html#autotoc_md63", [
        [ "📋 <b>Version History &amp; Release Notes</b>", "index.html#autotoc_md64", null ]
      ] ],
      [ "<b>v3.0.0 - Phase 1: Enhanced Control Systems</b> 🚀", "index.html#autotoc_md66", [
        [ "🎮 <b>Major Features: Joystick Curves &amp; Driver Control</b>", "index.html#autotoc_md67", [
          [ "<b>Advanced Joystick Curve System</b>", "index.html#autotoc_md68", null ],
          [ "<b>Configurable Drive Methods</b>", "index.html#autotoc_md69", null ],
          [ "<b>Performance Characteristics</b>", "index.html#autotoc_md70", null ]
        ] ],
        [ "💾 <b>Major Features: SD Card Data Persistence</b>", "index.html#autotoc_md72", [
          [ "<b>Completely Optional Architecture</b>", "index.html#autotoc_md73", null ],
          [ "<b>Tuning Profile Management</b>", "index.html#autotoc_md74", null ],
          [ "<b>Adaptive PID History Logging</b>", "index.html#autotoc_md75", null ],
          [ "<b>Match Data Recording</b>", "index.html#autotoc_md76", null ],
          [ "<b>Storage Management</b>", "index.html#autotoc_md77", null ]
        ] ],
        [ "🎯 <b>Major Features: LemLib-Style Movement Parameters</b>", "index.html#autotoc_md79", [
          [ "<b>Enhanced Parameter Structures</b>", "index.html#autotoc_md80", null ],
          [ "<b>Advanced Movement Control</b>", "index.html#autotoc_md81", null ],
          [ "<b>Movement Monitoring Framework</b>", "index.html#autotoc_md82", null ]
        ] ],
        [ "🏥 <b>Major Features: System Health Monitoring</b>", "index.html#autotoc_md84", [
          [ "<b>Comprehensive Health Detection</b>", "index.html#autotoc_md85", null ],
          [ "<b>Real-time Health System</b>", "index.html#autotoc_md86", null ],
          [ "<b>Health Status Classification</b>", "index.html#autotoc_md87", null ],
          [ "<b>Motor Health Metrics</b>", "index.html#autotoc_md88", null ]
        ] ]
      ] ],
      [ "<b>v2.1.0 - Adaptive Control &amp; Auto-Tuning</b> 🧠", "index.html#autotoc_md90", [
        [ "🎛️ <b>Adaptive PID Controller</b>", "index.html#autotoc_md91", null ],
        [ "🎯 <b>Auto-Tuning System</b>", "index.html#autotoc_md92", null ],
        [ "🛡️ <b>Safety Features</b>", "index.html#autotoc_md93", null ]
      ] ],
      [ "<b>v2.0.0 - Core Architecture Overhaul</b> 🏗️", "index.html#autotoc_md95", [
        [ "🔧 <b>Major Architecture Changes</b>", "index.html#autotoc_md96", null ],
        [ "📊 <b>Enhanced Odometry System</b>", "index.html#autotoc_md97", null ],
        [ "🎮 <b>Controller Integration</b>", "index.html#autotoc_md98", null ]
      ] ],
      [ "<b>v1.5.0 - MCL &amp; Localization</b> 🧭", "index.html#autotoc_md100", [
        [ "🎯 <b>Monte Carlo Localization</b>", "index.html#autotoc_md101", null ],
        [ "📍 <b>Pose Estimation</b>", "index.html#autotoc_md102", null ]
      ] ],
      [ "<b>v1.0.0 - Initial Release</b> 🎉", "index.html#autotoc_md104", [
        [ "🏗️ <b>Core Architecture</b>", "index.html#autotoc_md105", null ],
        [ "🎛️ <b>PID Control System</b>", "index.html#autotoc_md106", null ],
        [ "📏 <b>Geometry System</b>", "index.html#autotoc_md107", null ]
      ] ],
      [ "<b>📈 Performance Metrics &amp; Benchmarks</b>", "index.html#autotoc_md109", [
        [ "<b>v3.0.0 Performance Characteristics</b>", "index.html#autotoc_md110", null ],
        [ "<b>Optimization Achievements</b>", "index.html#autotoc_md111", null ]
      ] ],
      [ "<b>🔧 API Changes &amp; Breaking Changes</b>", "index.html#autotoc_md113", [
        [ "<b>v3.0.0 Breaking Changes</b>", "index.html#autotoc_md114", null ],
        [ "<b>Migration Guide</b>", "index.html#autotoc_md115", null ]
      ] ],
      [ "<b>🐛 Known Issues &amp; Limitations</b>", "index.html#autotoc_md117", [
        [ "<b>Current Limitations</b>", "index.html#autotoc_md118", null ],
        [ "<b>Resolved Issues</b>", "index.html#autotoc_md119", null ]
      ] ],
      [ "<b>🗺️ Future Roadmap (Detailed)</b>", "index.html#autotoc_md121", [
        [ "<b>Phase 2: Advanced Motion Control (Q4 2025)</b>", "index.html#autotoc_md122", null ],
        [ "<b>Phase 3: Sensor Fusion &amp; AI (Q1 2026)</b>", "index.html#autotoc_md123", null ],
        [ "<b>Phase 4: Competition Features (Q2 2026)</b>", "index.html#autotoc_md124", null ],
        [ "<b>Phase 5: Developer Experience (Q3 2026)</b>", "index.html#autotoc_md125", null ]
      ] ],
      [ "<b>📚 Configuration Examples</b>", "index.html#autotoc_md127", [
        [ "<b>Complete Robot Configuration</b>", "index.html#autotoc_md128", null ],
        [ "<b>Performance Tuning Configuration</b>", "index.html#autotoc_md129", null ]
      ] ],
      [ "<b>🤝 Contributing &amp; Development</b>", "index.html#autotoc_md131", [
        [ "<b>Development Setup</b>", "index.html#autotoc_md132", null ],
        [ "<b>Code Standards</b>", "index.html#autotoc_md133", null ],
        [ "<b>Testing Requirements</b>", "index.html#autotoc_md134", null ]
      ] ],
      [ "<b>📞 Support &amp; Resources</b>", "index.html#autotoc_md136", [
        [ "<b>Documentation</b>", "index.html#autotoc_md137", null ],
        [ "<b>Community</b>", "index.html#autotoc_md138", null ]
      ] ],
      [ "<b>🏆 Acknowledgments</b>", "index.html#autotoc_md140", [
        [ "<b>Contributors</b>", "index.html#autotoc_md141", null ],
        [ "<b>Inspirations</b>", "index.html#autotoc_md142", null ],
        [ "<b>Special Thanks</b>", "index.html#autotoc_md143", null ]
      ] ],
      [ "📚 License / Attribution", "index.html#autotoc_md146", [
        [ "Final Notes", "index.html#autotoc_md148", null ]
      ] ]
    ] ],
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
        [ "Functions", "functions_func.html", null ],
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
"classls_1_1DataLogger.html#ae974f1ad8bd9726ebb00f6242c360a09",
"classls_1_1SmartPID.html#acb149cc5cd7bb78040cbab45c76d691e",
"functions_vars_d.html",
"structls_1_1DataLogger_1_1AdaptiveData.html#a56fc016307789283a629e216dc443aec",
"structls_1_1SystemHealthMonitor_1_1SystemMetrics.html#a6bdca725d69dedaf7c0d5781be07b67a"
];

var SYNCONMSG = 'click to disable panel synchronization';
var SYNCOFFMSG = 'click to enable panel synchronization';
var LISTOFALLMEMBERS = 'List of all members';