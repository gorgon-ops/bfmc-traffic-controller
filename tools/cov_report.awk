# Turns "gcov -n" output into the table that goes in the README.
#
# gcov prints a File line followed by a Lines line for each source, and then
# one final unlabelled Lines line holding the total across all of them. The
# pairing is the whole trick: a Lines line that follows a File line belongs to
# that file, and one that does not is the total. Attributing the total to
# whichever file came last inflates that file and double-counts everything,
# which is exactly the bug this file was written to fix.
#
#   awk -f tools/cov_report.awk build/cov/gcov.txt

/^File/ {
    n = split($2, parts, "/")
    f = parts[n]
    gsub(/[^a-zA-Z0-9_.]/, "", f)      # strip the quotes and any stray CR
    next
}

/^Lines executed/ {
    split($0, a, ":")
    split(a[2], b, "%")
    split($0, c, "of ")
    pct   = b[1] + 0
    lines = c[2] + 0

    if (f != "") {
        printf "  %-22s %6.2f%% of %d\n", f, pct, lines
        f = ""
    } else {
        total_pct   = pct
        total_lines = lines
    }
}

END {
    if (total_lines > 0) {
        printf "  %-22s %6.2f%% of %d\n", "TOTAL", total_pct, total_lines
    }
}
