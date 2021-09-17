# gnuplot -e "filename='robot1_traj.txt'" plot3d.plt
set view equal xyz
splot filename using 1:2:3 title "ground-truth" with lines
replot filename using 4:5:6 title "estimated" with lines
set xlabel "x"; set ylabel "y"; set zlabel "z"
set view ,,2.0
set yrange [*:40]
set xrange [-10:25]
set ztics 2
pause -1 "Hit any key to continue"
