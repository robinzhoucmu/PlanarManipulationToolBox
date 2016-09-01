listing = dir('pushing_data/rect1_json/*v=300_i*.json');
for i = 1:1:length(listing)
    if listing(i).bytes < 100
        delete(strcat('pushing_data/rect1_json/',listing(i).name));
    end
end
