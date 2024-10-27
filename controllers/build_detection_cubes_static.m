function boxes = build_detection_cubes_static(q_row, links, base_transform, p_dc_t)
            
            boxes = cell(1,6);
            num_links = length(links); 
            %[~, transforms] = self.robot.model.fkine(q); 
            %reimplement fkine here to get current transforms
            transforms = createArray(4,4,width(q_row),"double");

            transformation = base_transform;

            function transform = a_copy(link, q) 
                sa = sin(link.alpha);
                ca = cos(link.alpha);
                if link.flip
                    q = -q + link.offset;
                else
                    q = q + link.offset;
                end

                % if link.isrevolute
                    % revolute (always)
                    st = sin(q); ct = cos(q);
                    d = link.d;
                % else
                %     % prismatic
                %     st = sin(link.theta); ct = cos(link.theta);
                %     d = q;
                % end

                %if L.mdh == 0
                % standard DH, always using standard DH so we can cut out
                % an if else statement for modified DH

                transform = [    ct  -st*ca  st*sa   link.a*ct
                    st  ct*ca   -ct*sa  link.a*st
                    0   sa      ca      d
                    0   0       0       1];
                %end
            end

            for i = 1:num_links
                transformation = transformation * a_copy(links(i), q_row(i));
                transforms(:,:,i) = transformation;
            end

            %transforms should be defined somewhere BEFORE this
            %transforms = transforms.T;
            %scale transforms
            %scalelinks = copy(links);
            for i = 1:num_links
               
                dc = DetectionController; %fake controller
                boxes{i} = DetectionCube(dc, transforms(:,:,i) * p_dc_t(:,:,i));
                 %= DetectionCube(dc,transforms(:,:,i));
            end
        end