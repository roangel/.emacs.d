(add-hook 'after-init-hook 'global-company-mode)
(use-package company-tabnine :ensure t)

;; Trigger completion immediately.
(setq company-idle-delay 0)

;; Number the candidates (use M-1, M-2 etc to select completions).
(setq company-show-numbers t)

(add-to-list 'company-backends #'company-tabnine)


;; workaround for company-transformers
(setq company-tabnine--disable-next-transform nil)
(defun my-company--transform-candidates (func &rest args)
(if (not company-tabnine--disable-next-transform)
    (apply func args)
    (setq company-tabnine--disable-next-transform nil)
    (car args)))

(defun my-company-tabnine (func &rest args)
(when (eq (car args) 'candidates)
    (setq company-tabnine--disable-next-transform t))
(apply func args))

(advice-add #'company--transform-candidates :around #'my-company--transform-candidates)
(advice-add #'company-tabnine :around #'my-company-tabnine)

;; Anaconda setup. Apart from autocomplete, also jumps to definition etc
(use-package anaconda-mode
  :ensure t
  :config
  (add-hook 'python-mode-hook 'anaconda-mode)
  ;;(add-hook 'python-mode-hook 'anaconda-eldoc-mode)
  )

;; Not used
;; (use-package irony-eldoc
;; :ensure t
;; :config
;; (add-hook 'irony-mode-hook #'irony-eldoc))

(provide 'my_autocompletion)
