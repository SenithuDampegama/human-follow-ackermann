🤖 Human-Follow Ackermann Robot

A vision-driven crawler that follows humans using AI, built on Jetson Orin Nano + Teensy 4.1.



https://github.com/user-attachments/assets/898d83e0-15fd-48f5-8a15-579c162862d4



✨ Features

🔍 Vision AI – DepthAI/PiNSIGHT camera running MobileNet-SSD

🧠 Layered architecture – Jetson for decision-making, Teensy for safety & actuation

🛡 Built-in safety – stall detection, impulse PWM, LiPo protection

🔌 Simple protocol – STOP / DRIVE / ARC / CENTER / PARAM with OK/DONE/TELEM

🛠 Fully documented – wiring table, BOM, CAD, and report included

📂 Repository Layout
firmware/     → Teensy firmware + libraries (C++)
jetson/       → Python code (vision, interface, decision layers)
cad/          → STEP files (Fusion 360 exports)
docs/         → Report, BOM, wiring, technical drawings
media/        → Images, renders, GIFs (for README showcase)

🧩 System Architecture

<img width="985" height="955" alt="human_follow_robot_architecture" src="https://github.com/user-attachments/assets/062f0b5e-b744-4acf-b0fd-9e938d87b004" />




🚀 Quickstart
Hardware

Teensy 4.1

Jetson Orin Nano 8GB

BTS7960 (drive), BTS7980 (steering)

DepthAI/PiNSIGHT camera

Dual 3S LiPos


📜 License

MIT License – free to use, modify, and build upon.
