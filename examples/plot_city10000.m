clear;

gt = dlmread('Data/ISAM2_GT_city10000.txt');

eh_poses = dlmread('../build/examples/ISAM2_TEST_city10000.txt');

figure;
hold on;
axis equal;
axis([-65 65 -75 60])

plot(gt(:,1), gt(:,2), '-', 'LineWidth', 4, 'color', [0.1 0.7 0.1]);

plot(eh_poses(:,1), eh_poses(:,2), '-', 'LineWidth', 2, 'color', [0.1 0.1 0.9]);
