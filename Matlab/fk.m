function [work_space] = fk(joint_space) 
    d_shoulder_to_center_link_1 = 0;
    a_link_1 = 1;
    d_center_l1_to_center_l2 = 0;
    a_link_2 = 1;
    d_center_l2_to_center_wrist2 = 0;
    d_dist_wrist1_ee = 0.2;


    dhps = zeros(5,4);
    dhps(1,:) = [d_shoulder_to_center_link_1, joint_space(1), a_link_1, pi()];
    dhps(2,:) = [d_center_l1_to_center_l2, joint_space(2), a_link_2, pi()];
    dhps(3,:) = [d_center_l2_to_center_wrist2, joint_space(3), 0, 0];
    dhps(4,:) = [0 pi()/2 0 pi()/2];
    dhps(5,:) = [d_dist_wrist1_ee, joint_space(4), 0, 0];

    ee = compose_dh_transforms(dhps);
    work_space = [ee(1:2,4)' joint_space(1)-joint_space(2)+joint_space(3) joint_space(4)];
end

function [transpose] = compose_dh_transforms(dhp) 
    [num_dhps, ~] = size(dhp);
    transpose = eye(4);
    for i = 1:num_dhps
        transpose = transpose * make_dh_transform(dhp(i,:));
    end
end

function [transform] = make_dh_transform(dhp)
    d = dhp(1);
    theta = dhp(2);
    a = dhp(3);
    alpha = dhp(4);


    d_transform = eye(4);
    d_transform(3,4) = d;
    
    theta_transform = eye(4);
    theta_transform(1:3,1:3) = rotz(theta * 180 / pi());
    
    a_transform = eye(4);
    a_transform(1,4) = a;
    
    alpha_transform = eye(4);
    alpha_transform(1:3,1:3) = rotx(alpha * 180 / pi());
                   
    transform = d_transform ...
                * theta_transform ...
                * a_transform ...
                * alpha_transform;
end