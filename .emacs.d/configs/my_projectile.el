(require 'projectile)
(define-key projectile-mode-map (kbd "C-c C-p") 'projectile-command-map)
(projectile-mode +1)
(provide 'my_projectile)
