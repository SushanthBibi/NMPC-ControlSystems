function save_fig(fig, figdir, name)
print(fig, fullfile(figdir, [name '.png']), '-dpng', '-r150');
end
