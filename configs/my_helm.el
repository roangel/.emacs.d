(require 'helm)
;; Use helm-M-x instead of M-x. No need for smex
(global-set-key (kbd "M-x") 'helm-M-x)
(global-set-key (kbd "C-x r b") 'helm-filtered-bookmarks)

;; No need for dired+ with helm find files
(global-set-key (kbd "C-x C-f") 'helm-find-files)

;; Autoresize and limit size of helm windows
(helm-autoresize-mode 1)
(setq helm-autoresize-max-height 25)
(setq helm-autoresize-min-height 20)

;; Helm inside the current window, not on bottom
(setq helm-split-window-in-side-p t)

;; Helm also for buffer list
(global-set-key (kbd "C-x C-b") 'helm-buffers-list)
(setq helm-buffer-max-length nil)

(require 'helm-projectile)
(helm-projectile-on)

(provide 'my_helm)
