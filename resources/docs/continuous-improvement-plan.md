# PhysicalMCP: Continuous Improvement Plan

> Prioritized plan for growing stars, users, contributors, and revenue.

---

## Immediate Blocker

```bash
npm login          # Re-authenticate (token expired)
cd packages/mcp-server
npm publish --access public
```

Without this, `npx @ricardothe3rd/physical-mcp` doesn't work and the README install command is broken.

---

## Track 1: Visibility (Gets Stars)

### Weekly Content — Pick 1 Per Week
- Blog post on Dev.to / Medium (draft exists in `resources/docs/blog-post-devto.md`)
- Short demo video (2-5 min) showing Claude controlling TurtleBot in Gazebo
- Twitter/X thread breaking down one safety feature with visuals
- Answer robotics questions on Stack Overflow / ROS Answers mentioning PhysicalMCP where relevant

### Monthly Launches
- Post each significant release on HN as "Show HN"
- Cross-post to r/ROS, r/robotics, r/LocalLLaMA, r/MachineLearning
- Post on ROS Discourse (the official ROS community)

---

## Track 2: Product (Gets Users + Contributors)

Build in this priority order:

| Priority | Feature | Why | Effort |
|----------|---------|-----|--------|
| 1 | **Record a demo GIF/video** | README with visuals gets 3-5x more stars | 2 hours |
| 2 | **"good first issue" labels** | Create 10-20 issues tagged for new contributors | 1 hour |
| 3 | **More robot policies** | UR5, Spot, Franka, TurtleBot4, drone profiles | 1 day |
| 4 | **OpenAI/Gemini integration example** | Shows it's not Claude-only, widens audience | 1 day |
| 5 | **Simple web dashboard** | Display audit events in a browser (even localhost) | 2-3 days |
| 6 | **GitHub Actions badge in README** | Show real CI passing live | 30 min |
| 7 | **npx interactive setup wizard** | `npx physical-mcp init` generates policy YAML | 1 day |
| 8 | **Gazebo demo world** | Custom world with obstacles to show geofence/collision | 1 day |

---

## Track 3: Revenue (Gets Money)

Lay the foundation now, execute later:

1. **GitHub Sponsors** — set up tiers at github.com/sponsors/ricardothe3rd (FUNDING.yml already wired)
2. **Collect emails** — add a "Get notified about PhysicalMCP Cloud" link to README footer
3. **Talk to robotics teams** — reach out to 5-10 ROS2 users, ask what they'd pay for
4. **Cloud dashboard** — comes after Track 2 item 5 proves demand

### Sponsor Tiers (When Ready)
| Tier | Price | Perks |
|------|-------|-------|
| Supporter | $9/mo | Name in README, Discord role |
| Startup | $49/mo | Priority issues, early feature access |
| Production | $149/mo | 1hr/month architecture call, private channel |
| Enterprise Sponsor | $499/mo | Logo on website, roadmap influence |

---

## What Drives GitHub Stars (Reference)

- Great README with visuals (done)
- Demo video / GIF (not done — highest priority)
- Regular releases and activity (green commit graph)
- Blog posts and content marketing
- Being featured on HN, Reddit, ROS Discourse
- Solving a real problem people actively search for
- Well-scoped "good first issue" labels for contributors

---

## Actionable Next Steps (In Order)

1. [ ] Fix npm token and publish v1.0.0
2. [ ] Record demo GIF/video for README
3. [ ] Post existing blog draft to Dev.to
4. [ ] Create 10-15 "good first issue" tickets on GitHub
5. [ ] Post "Show HN: PhysicalMCP" on Hacker News
6. [ ] Cross-post to r/ROS, r/robotics, r/LocalLLaMA
7. [ ] Post on ROS Discourse
8. [ ] Set up GitHub Sponsors tiers
9. [ ] Write UR5/Spot/drone safety policy files
10. [ ] Add OpenAI/Gemini integration examples to docs
11. [ ] Build simple localhost web dashboard for audit events
12. [ ] Add "Get notified about PhysicalMCP Cloud" email collection

---

*Last updated: 2026-02-23*
