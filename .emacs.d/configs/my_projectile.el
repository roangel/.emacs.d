(require 'projectile)
(define-key projectile-mode-map (kbd "C-c C-p") 'projectile-command-map)
(setq projectile-completion-system 'ivy)
(setq projectile-indexing-method 'hybrid)
(setq projectile-sort-order 'recently-active)
(setq projectile-enable-caching t)

(projectile-mode +1)
(provide 'my_projectile)
