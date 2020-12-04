set size ratio -1
set xlabel "y"
set ylabel "z"
set xrange [*:40]
clear
plot "robot1_multi_traj.txt" using 2:3 title "ground-truth" with lines
replot "robot1_multi_traj.txt" using 5:6 title "estimated" with lines
pause -1 "Hit any key to continue"
clear
plot "robot3_multi_traj.txt" using 2:3 title "ground-truth" with lines
replot "robot3_multi_traj.txt" using 5:6 title "estimated" with lines
pause -1 "Hit any key to continue"
clear
plot "robot1_single_traj.txt" using 2:3 title "ground-truth" with lines
replot "robot1_single_traj.txt" using 5:6 title "estimated" with lines
pause -1 "Hit any key to continue"

