function c_wrist = calc_wrist_CoM(m_link1, m_servo, m_link2, m_gripper,...
                                d_link1, d_servo, d_link2, d_gripper)
%calc_link_CoM Calculates the center of mass of a specific multi-joint link.
%   This function assumes that the robot is equipped with a gripper as its
%   6th joint and that only the first joint in the wrist will need to
%   compensate for a force on its center of mass. It models each servo as a
%   point mass and each link as a rod of uniform density. The gripper is
%   assumed to have a center of mass in the center of its length.
%   
%   INPUTS:
%       m_link1 - The mass of the link between joints 4 and 5
%       m_servo - The mass of the servo for joint 5
%       m_link2 - The mass between joint 5 and the gripper
%       m_gripper - The mass of the gripper
%       d_link1 - The length of the rod between joints 4 and 5
%       d_servo - The length of the servo for joint 5 (can be 0)
%       d_link2 - The length of the rod between joint 5 and the gripper
%       d_gripper - The length of the gripper
%   OUTPUTS:
%       c_wrist - The center of mass of the linkage. Note that this is a
%                scalar value and will have to be rotated and translated 
%                appropriately depending on its position in the robot arm.
%%
c_link1 = d_link1 / 2;
c_servo = d_link1 + (d_servo/2);
c_link2 = d_link1 + d_servo + (d_link2/2);
c_grip = d_link1 + d_servo + d_link2 + (d_gripper/2);
c_wrist = (c_link1*m_link1 + c_servo*m_servo + c_link2+m_link2 ...
           + c_grip*m_gripper) / (m_link1 + m_servo + m_link2 + m_gripper);
end

