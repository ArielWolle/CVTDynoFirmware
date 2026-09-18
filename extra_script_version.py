# Injects two build-time identifiers the firmware reports back over USB (see printCurrentConfig()
# in src/main.cpp) so the viewer can tell -- on every connect, with zero manual bookkeeping -- both
# exactly which commit is actually flashed AND whether it's protocol-compatible with itself. This
# exists because a stale-firmware-vs-viewer mismatch already caused real, hard-to-diagnose behavior
# (RPM never reaching zero) that took a live debugging session to track down before this existed.
#
#   FIRMWARE_GIT_SHA      -- short git commit hash of the exact source this binary was built from
#                            (falls back to "unknown" outside a git checkout, e.g. a source zip).
#                            Purely informational/for humans -- CI's firmware-<sha> artifact naming
#                            already uses the same identifier, so this ties a running device
#                            directly back to a specific CI build.
#   FIRMWARE_GIT_DIRTY     -- "+dirty" suffix appended when the working tree had uncommitted changes
#                            at build time, so a locally-flashed dev build is visibly distinguishable
#                            from a clean CI build of the same commit.
#
# See PROTOCOL_VERSION in main.cpp for the actual compatibility check the viewer performs --
# that one is a manually-bumped integer (bump it whenever a change alters wire-level semantics the
# viewer must know about), not derived from git, since "the commit changed" and "the wire protocol
# changed in a way older/newer viewers can't tolerate" are different questions.
import subprocess

Import("env")


def _run_git(args):
    try:
        return subprocess.check_output(
            ["git"] + args, stderr=subprocess.DEVNULL, cwd=env["PROJECT_DIR"]
        ).decode().strip()
    except Exception:
        return None


git_sha = _run_git(["rev-parse", "--short=10", "HEAD"]) or "unknown"
dirty = _run_git(["status", "--porcelain"])
git_sha_display = git_sha + ("+dirty" if dirty else "")

env.Append(CPPDEFINES=[("FIRMWARE_GIT_SHA", '\\"%s\\"' % git_sha_display)])
