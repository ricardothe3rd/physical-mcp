# Governance

## Project Structure

PhysicalMCP is maintained by [@ricardothe3rd](https://github.com/ricardothe3rd) with community contributions.

## Roles

### Maintainer

- Full commit access and release authority
- Reviews and merges pull requests
- Sets project direction and roadmap
- Manages releases and npm publishing

### Contributor

- Anyone who submits a pull request, issue, or discussion
- Contributors who make sustained, high-quality contributions may be invited to become maintainers

## Decision Making

- **Minor changes** (bug fixes, docs, tests): Maintainer merges directly
- **New features**: Discussed in GitHub Issues or Discussions before implementation
- **Breaking changes**: Require an RFC (request for comments) in GitHub Discussions with a minimum 7-day comment period
- **Safety-related changes**: Require review from at least one maintainer and explicit test coverage for all safety invariants

## Release Process

1. All tests must pass on CI (Node 18, 20, 22)
2. CHANGELOG.md updated with release notes
3. Version bumped in package.json
4. Git tag created matching the version
5. GitHub Actions publishes to npm automatically on tag push

## Code of Conduct

All participants are expected to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Safety Philosophy

Safety is the core differentiator of PhysicalMCP. Any change that weakens safety guarantees (velocity limits, geofence, e-stop, audit logging) will not be accepted without a compelling justification and comprehensive test coverage proving the safety invariants still hold.

## Contact

- GitHub Issues: Bug reports and feature requests
- GitHub Discussions: Questions, ideas, and general conversation
- Security issues: See [SECURITY.md](SECURITY.md) for responsible disclosure
