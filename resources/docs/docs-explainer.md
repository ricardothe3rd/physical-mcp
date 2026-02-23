# PhysicalMCP Documentation Explainer

> What each planning doc covers, why it matters, and how they connect.

---

## 1. `phases-2-3-4-roadmap.md` — The Big Picture

**What it is:** Your future product roadmap after Phase 1 (which is done).

**Phase 2 (Cloud Bridge)** — Making it work over the internet
- Right now PhysicalMCP only works locally (`ws://localhost:9090`)
- This phase adds a cloud relay so an AI agent in the cloud can control a robot anywhere in the world
- Includes user accounts, API keys, role-based access (who can control what)
- A web dashboard to monitor all your connected robots

**Phase 3 (Device Library + Marketplace)** — Growing the ecosystem
- Pre-built configs for popular robots (UR5 arms, Spot, DJI drones)
- A marketplace where people sell/share reusable robot skills ("pick up object", "scan room")
- Enterprise features: SSO, compliance reports, fleet management

**Phase 4 (Revenue)** — How you make money
- Pricing tiers: Free (open source) → $29/robot/month (cloud) → $149/robot (pro) → custom enterprise
- GitHub Sponsors tiers ($9 to $499/month)
- Comparable companies and what they charge (Foxglove, Balena, ThingsBoard)
- Realistic revenue timeline: $0-500/month at month 3, $5K-20K/month at month 12

**Why it matters:** So you don't forget the plan. When you're deep in code, you can re-read this to stay focused on where the money comes from.

---

## 2. `continuous-improvement-plan.md` — Your Weekly Playbook

**What it is:** A prioritized checklist of what to do to grow the project.

**Track 1: Visibility** — Getting eyeballs
- Write 1 blog post or make 1 demo video per week
- Post on HN, Reddit (r/ROS, r/robotics), ROS Discourse monthly
- Answer robotics questions on Stack Overflow and mention PhysicalMCP

**Track 2: Product** — What to build next, ranked 1-8
1. Record a demo GIF (READMEs with visuals get 3-5x more stars)
2. Create "good first issue" labels (attracts contributors)
3. More robot policies (UR5, Spot, drones)
4. OpenAI/Gemini examples (proves it's not Claude-only)
5. Web dashboard
6. CI badges
7. Setup wizard
8. Gazebo demo world

**Track 3: Revenue** — Laying the foundation
- Set up GitHub Sponsors
- Collect emails for "PhysicalMCP Cloud" waitlist
- Talk to 5-10 robotics teams about what they'd pay for

**The actionable checklist at the bottom** — 12 items in order, checkboxes ready to tick off.

**Why it matters:** Without this, you'll build random features. This keeps you focused on what actually moves the needle (stars → users → revenue).

---

## 3. `openai-gemini-integration.md` — How Other AI Models Connect

**What it is:** Step-by-step setup instructions for connecting PhysicalMCP to non-Claude AI models.

**OpenAI (3 ways):**
- **Agents SDK** — 10 lines of Python, runs locally, recommended
- **Responses API** — Direct API call, needs a public HTTPS endpoint
- **ChatGPT Desktop** — Connectors UI, also needs public endpoint, paid plans only

**Google Gemini (3 ways):**
- **Gemini CLI** — Add 1 JSON config file, works immediately, free
- **Google ADK** — 10 lines of Python, Google's agent framework
- **Gemini Python SDK** — Experimental but works, manual session wiring

**Any LLM (via mcp-use):**
- 5 lines of Python, works with OpenAI, Gemini, Ollama, or any LangChain model
- This is the universal adapter

**Editor/IDE support:**
- Cursor, Continue, Cline — all work with a simple `mcp.json` config file

**Tool filtering section** — How to limit which tools an AI can access (security best practice)

**Why it matters:** This is critical for adoption. If people think PhysicalMCP only works with Claude, you lose 70%+ of potential users. This doc proves it's universal. It should eventually become a page on your website/README.

---

## 4. `features-5-8-implementation-plan.md` — The Build Spec

**What it is:** Detailed technical blueprints for the next 4 features to build.

### Feature 5: Web Dashboard (2-3 days)
- An Express server running alongside the MCP server on `localhost:3000`
- Shows audit events in real-time (SSE streaming, no page refresh needed)
- Stats bar: total commands, allowed, blocked, errors
- Color-coded rows: green=allowed, red=blocked, yellow=warning
- Filter by type, result, time range
- Safety score gauge
- Single HTML file, no React/Vue needed — vanilla JS
- Triggered by `--dashboard` flag

ASCII mockup of the UI is included so you know exactly what to build.

### Feature 6: CI Badges (15 minutes)
- The CI badge already exists in README but needs CI to actually run
- Add npm download count and GitHub stars badges
- Just badge URL additions — trivial change

### Feature 7: Init Wizard (1 day)
- Run `npx @ricardothe3rd/physical-mcp init`
- Interactive prompts: pick robot type → set velocity limits → configure geofence → choose tool profile
- Built-in presets for TurtleBot3, UR5, drones, simulation
- Outputs a YAML policy file + JSON config for Claude/Cursor/Gemini
- Uses the `prompts` npm package (lightweight, 15KB)
- Wired in as a subcommand: `process.argv[2] === 'init'`

### Feature 8: Gazebo Demo World (1 day)
- Custom Gazebo world with walls matching the geofence boundary
- Obstacles inside for collision demos
- Narrow corridor to test collision zone warnings
- Matching `demo.yaml` policy file
- A 10-step demo script for recording the video

**Build order:**
```
Day 1: Badges (AM) → Init wizard (PM)
Day 2: Gazebo world
Day 3-4: Dashboard
Day 5: Record demo video
```

**Why it matters:** Without specs, you'll waste time figuring out architecture as you code. This lets you (or a contributor) just open the doc and start building. Every file to create and modify is listed.

---

## How They All Connect

```
continuous-improvement-plan.md    → WHAT to do (the strategy)
    │
    ├── features-5-8-plan.md      → HOW to build the next features
    │
    ├── openai-gemini-guide.md    → HOW to reach non-Claude users
    │
    └── phases-2-3-4-roadmap.md   → WHERE this is all going (the vision + money)
```

The improvement plan is your weekly compass. The feature spec is your build guide. The integration doc is your adoption multiplier. The roadmap is your north star.

---

## Full Documentation Index

| Doc | Purpose | Location |
|-----|---------|----------|
| Phases 2-4 Roadmap | Future product + revenue plan | `resources/docs/phases-2-3-4-roadmap.md` |
| Continuous Improvement Plan | Weekly growth playbook | `resources/docs/continuous-improvement-plan.md` |
| OpenAI + Gemini Integration | Multi-model setup guide | `resources/docs/openai-gemini-integration.md` |
| Features 5-8 Implementation | Technical build specs | `resources/docs/features-5-8-implementation-plan.md` |
| This Explainer | What each doc covers | `resources/docs/docs-explainer.md` |

---

*Last updated: 2026-02-23*
