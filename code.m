%Dynamic env
total = 10;     % no of ants
iterations = 1000;    %no. of iterations
phera_up = -1;
rho = 0.8;
alpha = 1;
beta = 0;

% initialization of env
mat = ones(250,250);
path = ones(250,250);
goal = [0 0];
[env, goal] = env_dy(mat,0,goal);
goal_a = zeros(1,2);
reached = -1;
reached_count = 0;
figure(1);

        % Initialization of ants
ant = zeros(total,iterations,2); 
ant_sta = zeros(total,2);
ant(:,1,:) = 1;
ant_sta(:,1) = 1;
phera = zeros(250,250);

        % Iterations
li =1:8; 
lm = [0 -1;1 -1;1 0;1 1;0 1;-1 1;-1 0;-1 -1];
lm = lm * 5;
filter = ones(5,5) * 0.25;           %gausian filter of size 3
filter(2:4,2:4) = ones(3,3) * 0.5;
filter(3,3) = 1;
for i = 1:iterations
    for j = 1:total
       if(ant_sta(j,2) == 1)
           continue;
       end
       min = -500; 
       r_temp = ant(j,ant_sta(j,1),1);
       c_temp = ant(j,ant_sta(j,1),2);
       l = li(randperm(8));
       for ll = 1:8
              r = ant(j,ant_sta(j,1),1)+lm(l(ll),1);
              c = ant(j,ant_sta(j,1),2)+lm(l(ll),2);
              if(r <= 0 || c <= 0 || r >= 251 || c >= 251 )
                 continue; 
              end
              
              if(r>=goal(1)-1 && r<=goal(1)+10 && c>=goal(2)-1 && c<=goal(2)+10)          %Goal Test
                  ant_sta(j,2) = 1;
                  r_temp = goal(1);
                  c_temp = goal(2);
                  goal_a(1) = r;
                  goal_a(2) = c;
                  reached_count = reached_count + 1;
                  if(reached == -1)
                    rho = 0.8;
                    reached = 1;
                    beta = 0;
                    alpha = 1;
                  end
                  for b=1:ant_sta(j,1)
                      a = ant_sta(j,1) + 1 -b;
                       if(ant(j,a,1) <=10 || ant(j,a,2) <=10 ||ant(j,a,1) >= 241 || ant(j,a,2) >= 241 )
                           phera(ant(j,a,1),ant(j,a,2)) = max(phera(ant(j,a,1),ant(j,a,2)) , 1/b);
                       else
                          temp_mat = filter*(1/b);
                          for u = 1:5
                             for v = 1:5
                                phera(ant(j,a,1)-15+5*u,ant(j,a,2)-15+5*v) = max(phera(ant(j,a,1)-15+5*u,ant(j,a,2)-15+5*v) , temp_mat(u,v));
                             end
                          end
                       end               
                  end                 
                  break;
              end
              
              if( env(r,c) == 1)
                  trans = (phera(r,c)/abs(phera(r,c)))*(abs(phera(r,c))^alpha)/(((sqrt((goal_a(1)-r)^2+(goal_a(2)-c)^2)+1))^beta);
                  if(trans < min)
                    continue;
                  end
                  min = trans;
                  r_temp = r;
                  c_temp = c;
              end
       end
       env(ant(j,ant_sta(j,1),1),ant(j,ant_sta(j,1),2)) = 1;
       env(r_temp,c_temp) = 0;
       if(~(ant(j,ant_sta(j,1),1) == r_temp && ant(j,ant_sta(j,1),2) == c_temp))
          ant_sta(j,1) = ant_sta(j,1) +1; 
       end
       ant(j,ant_sta(j,1),1) = r_temp;
       ant(j,ant_sta(j,1),2) = c_temp; 

       if(phera(r_temp,c_temp) <= 0)
           if(r_temp <=10 || c_temp <=10 ||r_temp >= 241 || c_temp >= 241 )
              phera(r_temp,c_temp) = phera(r_temp,c_temp) + phera_up;
           else
              temp_mat = filter*phera_up;
              for u = 1:5
                  for v = 1:5
                     phera(r_temp-15+5*u,c_temp-15+5*v) = phera(r_temp-15+5*u,c_temp-15+5*v) + temp_mat(u,v);
                  end
             end
           end
       end
       path(r_temp,c_temp) = 0;
    end 
    phera = phera * (rho);  
    imshow(env);
    pause(0.0001);
    if(reached_count == total)
        break;
    end
    [env, goal] = env_dy(mat,i,goal);
   
    for chiti = 1:total
       if(ant_sta(chiti,2) == 1 || env(ant(chiti,ant_sta(chiti,1),1),ant(chiti,ant_sta(chiti,1),2)) == 1)
           env(ant(chiti,ant_sta(chiti,1),1),ant(chiti,ant_sta(chiti,1),2)) = 0;
           continue;
       end
       min = -500; 
       r_temp = ant(chiti,ant_sta(chiti,1),1);
       c_temp = ant(chiti,ant_sta(chiti,1),2);
       l = li(randperm(8));
       for ll = 1:8
              r = ant(chiti,ant_sta(chiti,1),1)+lm(l(ll),1);
              c = ant(chiti,ant_sta(chiti,1),2)+lm(l(ll),2);
              if(r <= 0 || c <= 0 || r >= 251 || c >= 251 )
                 continue; 
              end
              if(r>=goal(1)-1 && r<=goal(1)+10 && c>=goal(2)-1 && c<=goal(2)+10)          %Goal Test
                  ant_sta(chiti,2) = 1;
                  r_temp = goal(1);
                  c_temp = goal(2);
                  goal_a(1) = r;
                  goal_a(2) = c;
                  reached_count = reached_count + 1;
                  if(reached == -1)
                    rho = 1;
                    reached = 1;
                    beta = 0;
                    alpha = 1;
                  end
                  for b=1:ant_sta(j,1)
                      a = ant_sta(j,1) + 1 -b;
                       if(ant(j,a,1) <=10 || ant(j,a,2) <=10 ||ant(j,a,1) >= 241 || ant(j,a,2) >= 241 )
                           phera(ant(j,a,1),ant(j,a,2)) = max(phera(ant(j,a,1),ant(j,a,2)) , 1/b);
                       else
                          temp_mat = filter*(1/b);
                          for u = 1:5
                             for v = 1:5
                                phera(ant(j,a,1)-15+5*u,ant(j,a,2)-15+5*v) = max(phera(ant(j,a,1)-15+5*u,ant(j,a,2)-15+5*v) , temp_mat(u,v));
                             end
                          end
                       end               
                  end                 
                  break;
              end
              
              if(env(r,c) == 1)
                  trans = reached*(abs(phera(r,c))^alpha)/(((sqrt((goal_a(1)-r)^2+(goal_a(2)-c)^2)+1))^beta);
                  if(trans < min)
                    continue;
                  end
                  min = trans;
                  r_temp = r;
                  c_temp = c;
              end
       end
       env(r_temp,c_temp) = 0;
       if(~(ant(chiti,ant_sta(chiti,1),1) == r_temp && ant(chiti,ant_sta(chiti,1),2) == c_temp))
          ant_sta(chiti,1) = ant_sta(chiti,1) +1; 
       else if(ant_sta(chiti,1) ~= 1&&(ant(chiti,ant_sta(chiti,1)-1,1) == r_temp && ant(chiti,ant_sta(chiti,1)-1,2) == c_temp))
          ant_sta(chiti,1) = ant_sta(chiti,1) -1;    
           end
       end
       ant(chiti,ant_sta(chiti,1),1) = r_temp;
       ant(chiti,ant_sta(chiti,1),2) = c_temp; 

       if(phera(r_temp,c_temp) <= 0)
           if(r_temp <=10 || c_temp <=10 ||r_temp >= 241 || c_temp >= 241 )
              phera(r_temp,c_temp) = phera(r_temp,c_temp) + phera_up;
           else
              temp_mat = filter*phera_up;
              for u = 1:5
                  for v = 1:5
                     phera(r_temp-15+5*u,c_temp-15+5*v) = phera(r_temp-15+5*u,c_temp-15+5*v) + temp_mat(u,v);
                  end
             end
           end
        end
        path(r_temp,c_temp) = 0;
    end

    
end
index = 1;
   for i=1:total
       if(ant_sta(i,2) == 1 && ant_sta(i,1) < ant_sta(index,1))
           index = i;
       end
   end
   
for i = 1: ant_sta(index,1)
    for ii = i+1:ant_sta(index,1)
        if(ant(index,i,1) == ant(index,ii,1) && ant(index,i,2) == ant(index,ii,2))
            for iii = 1:ant_sta(index,1)-i
               ant(index,i+iii,1) = ant(index,iii+ii,1);
               ant(index,i+iii,2) = ant(index,iii+ii,2);
            end
           ant_sta(index,1) = ant_sta(index,1) - ii +i;
        end
    end
end
   
for j=1:ant_sta(index,1)
   env(ant(index,j,1),ant(index,j,2)) = 0; 
   pause(0.1);
   figure(3);
   imshow(env);
end   


        
        
       


    
