test_plot = GenericRenderable('Bench.ply')
test_plot_pos = test_plot.set_brick_pose(-1.6, 0.8, -0.2, eye(3));
test_plot_patch = patch('Faces',test_plot.tri,'Vertices',test_plot_pos, 'FaceColor', [0 0.1 0.3], 'EdgeColor', 'none');