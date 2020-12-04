set size ratio -1
set xlabel "y"
set ylabel "x"
set xrange [*:40]
set yrange [*:*] reverse
clear
plot "robot1_multi_traj.txt" using 2:1 title "ground-truth" with lines
replot "robot1_multi_traj.txt" using 5:4 title "estimated" with lines
pause -1 "Hit any key to continue"
clear
plot "robot3_multi_traj.txt" using 2:1 title "ground-truth" with lines
replot "robot3_multi_traj.txt" using 5:4 title "estimated" with lines
pause -1 "Hit any key to continue"
clear
plot "robot1_single_traj.txt" using 2:1 title "ground-truth" with lines
replot "robot1_single_traj.txt" using 5:4 title "estimated" with lines
pause -1 "Hit any key to continue"

