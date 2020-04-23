function c_link = calc_link_CoM(m_link, m_servo, d_link)
%calc_link_CoM Calculates the center of mass of an arbitrary robot link
%   This function assumes that the link is modeled as a uniform rod with a
%   point mass on one end. It calculates the center of mass by
%   assuming the center of the rod is half of its length and that the
%   center of the servo is at the end of the rod, 
%   and then using the standard two-body center of mass equation to 
%   find the center of the entire linkage.
%   
%   INPUTS:
%       m_link - The mass of the link, or rod portion of the 2-body linkage
%       m_servo - The mass of the servo on the end of the linkage
%       d_link - The length of the rod
%       d_servo - The length of the servo
%   OUTPUTS:
%       c_link - The center of mass of the linkage. Note that this is a
%                scalar value and will have to be rotated and translated 
%                appropriately depending on its position in the robot arm.
%%
c_rod = d_link / 2;
c_servo = d_link;
c_link = (m_link*c_rod + m_servo*c_servo)/(m_link + m_servo);
end

