t_inst = 0;
t = [0];
light = [0];
temperature = [0];
out_value = [0];
flag = 0
while(1)
   [q,w] = system(['tail -n ','1',' ','/dev/ttyACM0']);
   treat = w
   pause(0.1)
   
   if(~(isempty(treat)))
       at_value = strsplit(treat);
       check_size = size(at_value);
       if(check_size(2)==5 && ~(isempty(at_value{1})) && ~(isempty(at_value{2})) && ~(isempty(at_value{3})) && ~(isempty(at_value{4})))
            if(str2num(at_value{4}) ~= t_inst)
                if (flag==0)
                    t=str2num(at_value{4})
                    light = str2num(at_value{1});
                    temperature = str2num(at_value{2});
                    out_value = str2num(at_value{3});
                    flag=1;
                   
                else
                    t_inst=str2num(at_value{4})
                    light = [str2num(at_value{1}) ;light];
                    temperature = [str2num(at_value{2}) ;temperature];
                    out_value = [str2num(at_value{3}) ;out_value];


                    t = [t_inst ; t];
                    display(at_value) 

                    subplot(3,1,1);
                    plot(t,light,':black');      
                    legend(strcat('Light: ',at_value{1}));

                    subplot(3,1,2);
                    plot(t,temperature,'--r');
                    legend(strcat('Temperature: ',at_value{2}));

                    subplot(3,1,3);
                    plot(t,out_value);
                    legend(strcat('Fan value: ',at_value{3}));

                    pause(0.1);
                end
            end
        end
    end
        
end