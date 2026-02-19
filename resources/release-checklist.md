# PhysicalMCP Release Checklist

## Pre-release

- [ ] All tests pass locally (`npm test` in `packages/mcp-server/`)
- [ ] CI pipeline is green on the main branch
- [ ] CHANGELOG.md is updated with all notable changes for this version
- [ ] Version bumped in `packages/mcp-server/package.json`
- [ ] README.md updated (new features, API changes, configuration options)
- [ ] Documentation in `resources/docs/` reflects any changes
- [ ] No outstanding critical or high-priority issues for this milestone

## Release

- [ ] Create and push a git tag: `git tag vX.Y.Z && git push origin vX.Y.Z`
- [ ] Verify the Release CI workflow triggers and completes successfully
- [ ] Verify the package is published to npm (`npm info @anthropic/physical-mcp`)
- [ ] Verify the GitHub Release is created with auto-generated release notes
- [ ] Verify Docker images are built and pushed to ghcr.io:
  - [ ] `ghcr.io/ricardothe3rd/physical-mcp-bridge:latest`
  - [ ] `ghcr.io/ricardothe3rd/physical-mcp-bridge:X.Y.Z`
  - [ ] `ghcr.io/ricardothe3rd/physical-mcp-sim:latest`
  - [ ] `ghcr.io/ricardothe3rd/physical-mcp-sim:X.Y.Z`

## Post-release

- [ ] Announce the release on social media
- [ ] Update GitHub Discussions with release highlights
- [ ] Monitor GitHub Issues for bug reports related to the new release
- [ ] Verify installation works cleanly: `npx @anthropic/physical-mcp`
- [ ] Test Docker images pull and run correctly from ghcr.io
