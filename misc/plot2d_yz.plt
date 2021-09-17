# gnuplot -e "filename='robot1_traj.txt'" plot2d_yz.plt
set size ratio -1
set xlabel "y"
set ylabel "z"
set xrange [*:40]
clear
plot filename using 2:3 title "ground-truth" with lines
replot filename using 5:6 title "estimated" with lines
pause -1 "Hit any key to continue"
