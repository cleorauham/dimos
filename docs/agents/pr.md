# Opening PRs

## Finding the Linear issue

When opening a PR for a GitHub issue, find the corresponding Linear issue to reference.
Issues are cloned automatically so titles match exactly.

1. Get the GitHub issue title:
   ```
   gh issue view <number> --repo dimensionalOS/dimos --json title --jq .title
   ```
2. Search Linear using the `@tacticlaunch/mcp-linear` MCP server (`linear_searchIssues` tool):
   ```
   linear_searchIssues query="<exact title>" limit=10
   ```
3. Match by **exact title** (not substring/fuzzy). If a match is found, use its identifier (e.g. `DIM-569`) in the PR description with `Closes DIM-XXX`.
4. If no matching Linear issue is found, write `No matching Linear issue found` instead of `Closes DIM-XXX`.

---

Below follows the PR description template you should use.

## Problem

<!-- What feature are you adding, or what is broken/missing/sub-optimal? -->
<!-- Context, symptoms, motivation. Link the issue. -->

Closes DIM-XXX <!-- or "No matching Linear issue found" -->

## Solution

<!-- What you changed  -->
<!-- Keep it high-signal; deep planning belongs in the issue. -->

## Breaking Changes

<!-- Write "None" if not applicable -->

<!-- If applicable:
- what breaks
- who is affected
- migration steps
-->

## How to Test

<!-- MUST be reproducible. If this section is weak, reviewers can't approve confidently. -->

## Contributor License Agreement

- [ ] I have read and approved the [CLA](https://github.com/dimensionalOS/dimos/blob/main/CLA.md).
