f = figure();
plot(t, squeeze(p(1:3,4,:)),'LineWidth', 3.); hold on;
plot(t, io.Data.xref, '--', 'LineWidth', 3.);
legend({'x','y','z'},'FontSize',15);
xlabel('Time [s]','FontSize',15)
ylabel('Cartesian position [m]','FontSize',15)
set(gca,'FontSize',15)
f.PaperPositionMode = 'auto';
print('Cartesian.eps', '-depsc');