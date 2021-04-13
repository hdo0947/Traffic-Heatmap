function plotbox = minboxplot(box_coord)
coord1 = box_coord(1,:);
coord2 = box_coord(2,:);
coord3 = box_coord(3,:);
coord4 = box_coord(4,:);
coord5 = box_coord(5,:);
coord6 = box_coord(6,:);
coord7 = box_coord(7,:);
coord8 = box_coord(8,:);

box_coord = [coord1; coord2; coord3; coord4; coord1; coord5; coord6; coord7; coord8; coord5];

hold on
plot3(box_coord(:,1), box_coord(:,2), box_coord(:,3),'b')
plot3([coord2(1,1) coord6(1,1)],[coord2(1,2) coord6(1,2)],[coord2(1,3) coord6(1,3)],'b')
plot3([coord3(1,1) coord7(1,1)],[coord3(1,2) coord7(1,2)],[coord3(1,3) coord7(1,3)],'b')
plot3([coord4(1,1) coord8(1,1)],[coord4(1,2) coord8(1,2)],[coord4(1,3) coord8(1,3)],'b')
end