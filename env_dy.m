function [mat,goal] = env_dy(mat,time,goal)

shapeInserter = vision.ShapeInserter('Fill',true,'Opacity',1);

size = int32(14);
if(mod(idivide(time,size),2) == 0) 
     x =  mod(time,size)*5 +6; 
else
     x = size*5 - mod(time,size)*5 +1; 
end
dy = [x,120,60,20; x+111,120,60,20];
rectangle = int32([dy;1 50 150 20; 201 50 50 20; 1 180 50 20; 101 180 150 20]);
mat = step(shapeInserter, mat, rectangle);

if(goal) 
    r = goal(1);
    c = goal(2);
else
    while(true)
        r = randi([1 241]);
        c = randi([1 241]);
        if(mat(r,c) == 1 && mat(r+9,c) == 1 && mat(r,c+9) == 1 && mat(r+9,c+9) == 1 && (r+10<120 || r>140))
           break;
        end
    end
end

goal = [r c];
rectangle = int32([c r 10 10]); 
mat = step(shapeInserter, mat, rectangle);

end

