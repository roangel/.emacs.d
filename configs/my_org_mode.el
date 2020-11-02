(require 'org)

(setq org-agenda-files (list "~/.org/my-org-files/work.org"
                             "~/.org/my-org-files/personal.org"))

(setq org-agenda-span 10
      org-agenda-start-on-weekday nil
      org-agenda-start-day "-5d")

(setq calendar-week-start-day 1)

(provide 'my_org_mode)
