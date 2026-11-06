# Test evidence

The data behind the results in Section 5 of the C5 final report. The two scripts recompute the report's tables from these files.

| Report result | Files | Recompute with |
|---|---|---|
| Rehearsal summary (Table 23, Figure 14) and position accuracy (Table 25, Figure 16) | `rehearsal_2026-11-04/logs/`, the sim log and timer output of all 10 Part 1 runs | `python3 evidence/rehearsal_2026-11-04/analyse.py` after `bash demo/get_world.sh` |
| Operator, Pause and PS4 runs | `rehearsal_2026-11-04/summary.md`, the notes from the 4 November rehearsal | |
| Fleet sizes (Table 24, Figure 15) | `fleet_sizes/bench_results.csv`, rows `*_lite_v6_dt2` and `*_lite_nat2_dt2` | |
| Rover detection range (Table 27) and guard stop distance (Table 28) | `final_tests_2026-11-06/r1.csv`, `r12.csv` | `python3 evidence/final_tests_2026-11-06/summarise.py` |
| Look-alike decoys (Table 29) | `final_tests_2026-11-06/decoys.json`, `decoy_run_sim.log.gz` | |
| Handover test (Section 6) | `section6_test_log.md`, every line run on a blank Ubuntu 22.04 sandbox | |

`r1_test.py`, `r12_test.py` and `decoy_make.py` are the scripts that ran those tests, kept as a record. They expect a running simulation.

The summary notes and test log were written during testing, so their paths point at the team laptop and some of their advice is about demo day. Odometry CSVs and Part 2 run logs (about 200 MB) are held by the team.
