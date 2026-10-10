import { useRef, type ComponentProps, type ReactNode } from "react";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "./ui/select";
import { Slider } from "./ui/slider";

type SelectOption = { value: string; label: ReactNode; disabled?: boolean };
type FormSelectProps = Omit<ComponentProps<typeof SelectTrigger>, "value" | "onChange" | "children"> & {
  value: string;
  onValueChange: (value: string) => void;
  options: readonly SelectOption[];
  placeholder?: string;
};

/** 여러 화면에서 포털의 배치와 키보드 조작을 공유하는 shadcn Select. */
export function FormSelect({ value, onValueChange, options, placeholder, disabled, ...props }: FormSelectProps) {
  const trigger=useRef<HTMLButtonElement>(null);
  return <Select value={value} onValueChange={onValueChange} disabled={disabled}>
    <SelectTrigger ref={trigger} {...props}><SelectValue placeholder={placeholder} /></SelectTrigger>
    <SelectContent position="popper" align="start" sideOffset={5} collisionPadding={12} onCloseAutoFocus={event=>{
      // 닫힘 애니메이션 중 사용자가 비행 화면/다른 컨트롤로 옮긴 초점을 되돌리지 않는다.
      const active=document.activeElement;
      if(active&&active!==document.body&&active!==trigger.current&&!active.closest('[data-slot="select-content"]'))event.preventDefault();
    }}>
      {options.map(option => <SelectItem key={option.value} value={option.value} disabled={option.disabled}>{option.label}</SelectItem>)}
    </SelectContent>
  </Select>;
}

type FormSliderProps = Omit<ComponentProps<typeof Slider>, "value" | "defaultValue" | "onValueChange"> & {
  value: number;
  onValueChange: (value: number) => void;
};

export function FormSlider({ value, onValueChange, ...props }: FormSliderProps) {
  return <div className="form-slider"><Slider {...props} value={[value]} onValueChange={values => onValueChange(values[0])} /></div>;
}
