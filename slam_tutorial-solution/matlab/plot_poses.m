function h = plot_poses(pose_data)

n_poses = size(pose_data, 1);

for i=1:n_poses
    id = pose_data(i, 1);
    q = pose_data(i, 2:5);
    t = pose_data(i, 6:8);
    
    draw_frame(q, t, 0.5);
    text(t(1), t(2), t(3), [num2str(id)]);
end

end

function draw_frame(q, t, s)

if min(size(q)) == 1
    q = reshape(q, [1,4]);
    R = quat2rotm(q);
end

if size(t, 1) == 1
    t = t';
end

C = repmat(t, 1, 3);
E = s * R + C;
plot3([t(1), E(1,1)],[t(2), E(2,1)],[t(3), E(3,1)],'-r','linewidth',2); hold on;
plot3([t(1), E(1,2)],[t(2), E(2,2)],[t(3), E(3,2)],'-g','linewidth',2);
plot3([t(1), E(1,3)],[t(2), E(2,3)],[t(3), E(3,3)],'-b','linewidth',2);

axis equal; grid on;

end
