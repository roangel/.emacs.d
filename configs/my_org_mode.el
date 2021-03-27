(require 'org)

(setq org-agenda-files (list "~/.org/my-org-files/work.org"
                             "~/.org/my-org-files/personal.org"))

(setq org-agenda-span 10
      org-agenda-start-on-weekday nil
      org-agenda-start-day "-5d")

(setq calendar-week-start-day 1)

(setq org-todo-keywords
  '((sequence
     "TODO(t!)" ; Initial creation
     "IN PROGRESS(i@)"; Work in progress
     "WAIT(w@)" ; My choice to pause task
     "BLOCKED(b@)" ; Not my choice to pause task
     "REVIEW(r!)" ; Inspect or Share Time
     "|" ; Remaining close task
     "DONE(d@)" ; Normal completion
     "CANCELED(c@)" ; Not going to od it
     "DUPLICATE(p@)" ; Already did it
     )))

(setq org-todo-keyword-faces (quote (
                                     ("TODO" :foreground "tomato")
                                     ("IN PROGRESS" :foreground "light slate gray")
                                     ("WAIT" :foreground "salmon1")
                                     ("BLOCKED" :foreground "orange")
                                     ("REVIEW" :foreground "turquoise")
                                     ("CANCELED" :foreground "gold")
                                     ("DUPLICATE" :foreground "orchid"))))

(with-eval-after-load 'org
  (setq org-startup-indented t) ; Enable `org-indent-mode' by default
  (add-hook 'org-mode-hook #'visual-line-mode))

(org-babel-do-load-languages
 'org-babel-load-languages
 '((emacs-lisp . t)
   (python . t)
   (C . t)))
(setq org-babel-python-command "python3")


(provide 'my_org_mode)
