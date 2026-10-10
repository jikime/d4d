import {Dialog as DialogPrimitive} from 'radix-ui';
import {X} from 'lucide-react';
import type {ComponentProps} from 'react';
import {t} from '../../i18n';

export const Dialog=DialogPrimitive.Root;
export const DialogTitle=DialogPrimitive.Title;
export const DialogDescription=DialogPrimitive.Description;
export function DialogContent({children,className='',...props}:ComponentProps<typeof DialogPrimitive.Content>){
  return <DialogPrimitive.Portal><DialogPrimitive.Overlay className="training-dialog-overlay"/><DialogPrimitive.Content data-slot="dialog-content" className={`training-dialog ${className}`} {...props}>{children}<DialogPrimitive.Close className="training-dialog-close" aria-label={t('닫기')}><X size={19}/></DialogPrimitive.Close></DialogPrimitive.Content></DialogPrimitive.Portal>;
}
