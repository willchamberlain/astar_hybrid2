X = -2:0.5:6.5;
Y = exp(X/1.1)/300;
y_lim = Y;  y_lim(y_lim>1)=1; y_lim(y_lim<0)=0;
figure_named('exp func'); plot(X,y_lim)

find(Y>=(1-0.2) & Y<=(1+0.2) )

X(find(Y>=(1-0.2) & Y<=(1+0.2) ))

prob_y = 1-Y;
prob_y_lim = prob_y;  prob_y_lim(prob_y_lim>1)=1; prob_y_lim(prob_y_lim<0)=0;
hold on;  plot(X,prob_y_lim); grid on