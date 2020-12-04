set view equal xyz
splot "robot1_multi_traj.txt" using 1:2:3 title "ground-truth" with lines
replot "robot1_multi_traj.txt" using 4:5:6 title "estimated" with lines
set xlabel "x"; set ylabel "y"; set zlabel "z"
set view ,,2.0
set yrange [*:40]
pause -1 "Hit any key to continue"
set view equal xyz
splot "robot3_multi_traj.txt" using 1:2:3 title "ground-truth" with lines
replot "robot3_multi_traj.txt" using 4:5:6 title "estimated" with lines
set xlabel "x"; set ylabel "y"; set zlabel "z"
set view ,,2.0
set yrange [*:40]
pause -1 "Hit any key to continue"
set view equal xyz
splot "robot1_single_traj.txt" using 1:2:3 title "ground-truth" with lines
replot "robot1_single_traj.txt" using 4:5:6 title "estimated" with lines
set xlabel "x"; set ylabel "y"; set zlabel "z"
set view ,,2.0
set yrange [*:40]
pause -1 "Hit any key to continue"

